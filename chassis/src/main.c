/**
 * @file main.c
 * @brief 底盘命令驱动控制 — 接收 VEL_CMD 控制电机, 发送 ODOM_DATA/STATUS/HEARTBEAT
 *
 * 下行 (RDK X5 → STM32): VEL_CMD @ 100Hz
 *   接收后转换为左右轮 PWM, 超时 200ms 无指令则自动刹车
 *
 * 上行 (STM32 → RDK X5):
 *   ODOM_DATA @ 50Hz (编码器+IMU数据)
 *   STATUS    @ 10Hz (底盘状态)
 *   HEARTBEAT @ 1Hz
 */

#include "stm32f10x.h"
#include "motor.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "encoder.h"
#include "protocol.h"

/* ── 运动学参数 ── */
#define MAX_PWM            999     /* PWM 最大值 (0~999) */
#define MAX_LINEAR_SPEED   0.5f    /* 最大线速度 m/s (与 ROS2 侧一致) */
#define MAX_ANGULAR_SPEED  1.0f    /* 最大角速度 rad/s */
#define WHEEL_BASE         0.35f   /* 左右轮间距 (m) — 需实测标定 */

/* ── 帧发送间隔 ── */
#define ODOM_INTERVAL_MS      20
#define STATUS_INTERVAL_MS   100
#define HEARTBEAT_INTERVAL_MS 1000

/* ── 方向校正 ──
 * 根据实际接线/机械安装调整:
 *   +1 = 正常, -1 = 反转
 * 前进时如果某侧轮子向后转, 就把该侧的 INVERT 改为 -1
 */
#define LEFT_INVERT   1     /* 左轮方向 */
#define RIGHT_INVERT  -1    /* 右轮方向 (实测: 右轮需反转) */

/* ── 指令超时 ── */
#define CMD_TIMEOUT_MS  200       /* 200ms 无新指令 → 刹车 */

/**
 * @brief 线速度+角速度 → 左右轮 PWM
 *
 * 差速运动学逆解:
 *   v_left  = v - ω * L/2
 *   v_right = v + ω * L/2
 *
 * 速度 → PWM: pwm = (speed / max_speed) * MAX_PWM
 */
static void vel_to_pwm(float linear_x, float angular_z,
                       int16_t *left_pwm, int16_t *right_pwm)
{
    float v_left  = linear_x - angular_z * WHEEL_BASE / 2.0f;
    float v_right = linear_x + angular_z * WHEEL_BASE / 2.0f;

    /* 限幅 */
    if (v_left  >  MAX_LINEAR_SPEED) v_left  =  MAX_LINEAR_SPEED;
    if (v_left  < -MAX_LINEAR_SPEED) v_left  = -MAX_LINEAR_SPEED;
    if (v_right >  MAX_LINEAR_SPEED) v_right =  MAX_LINEAR_SPEED;
    if (v_right < -MAX_LINEAR_SPEED) v_right = -MAX_LINEAR_SPEED;

    /* 速度 → PWM (带符号) */
    *left_pwm  = (int16_t)(v_left  / MAX_LINEAR_SPEED * MAX_PWM);
    *right_pwm = (int16_t)(v_right / MAX_LINEAR_SPEED * MAX_PWM);
}

/**
 * @brief 设置左右轮 PWM (自动处理方向和死区)
 */
static void set_motors(int16_t left_pwm, int16_t right_pwm)
{
    MotorDir dir_l, dir_r;
    uint16_t spd_l, spd_r;

    if (left_pwm >= 0)
    {
        dir_l = MOTOR_FORWARD;
        spd_l = (uint16_t)left_pwm;
    }
    else
    {
        dir_l = MOTOR_BACKWARD;
        spd_l = (uint16_t)(-left_pwm);
    }

    if (right_pwm >= 0)
    {
        dir_r = MOTOR_FORWARD;
        spd_r = (uint16_t)right_pwm;
    }
    else
    {
        dir_r = MOTOR_BACKWARD;
        spd_r = (uint16_t)(-right_pwm);
    }

    /* 小死区: <1% 占空比视为停止 */
    if (spd_l < 10) spd_l = 0;
    if (spd_r < 10) spd_r = 0;

    motor_set(MOTOR_LEFT,  dir_l, spd_l);
    motor_set(MOTOR_RIGHT, dir_r, spd_r);
}

/* ================================================================
 * main
 * ================================================================ */

int main(void)
{
    motor_init();
    bsp_delay_init();
    bsp_usart2_init();
    encoder_init();
    protocol_init();

    uint32_t last_odom      = 0;
    uint32_t last_status    = 0;
    uint32_t last_heartbeat = 0;
    uint32_t last_cmd       = 0;  /* 上次收到 VEL_CMD 的时间 */
    uint32_t now;

    float target_linear  = 0.0f;
    float target_angular = 0.0f;
    int cmd_received = 0;

    while (1)
    {
        now = bsp_delay_get_ms();

        /* ── 接收并处理下行帧 ── */
        uint8_t ftype;
        const uint8_t *payload;
        int plen;
        while (protocol_try_parse(&ftype, &payload, &plen))
        {
            if (ftype == 0x81)  /* VEL_CMD */
            {
                VelCmd cmd;
                if (protocol_parse_vel_cmd(payload, plen, &cmd))
                {
                    target_linear  = cmd.linear_x;
                    target_angular = cmd.angular_z;
                    last_cmd = now;
                    cmd_received = 1;
                }
            }
            /* 其他下行帧 (LED_CTRL, BUZZER, RESET_ODOM, MOTOR_BRAKE) 暂未处理 */
        }

        /* ── 指令超时检查 ── */
        if (cmd_received && (now - last_cmd > CMD_TIMEOUT_MS))
        {
            target_linear  = 0.0f;
            target_angular = 0.0f;
            cmd_received = 0;  /* 不再重复刹车 */
        }

        /* ── 执行速度指令 ── */
        int16_t left_pwm, right_pwm;
        vel_to_pwm(target_linear, target_angular, &left_pwm, &right_pwm);
        left_pwm  *= LEFT_INVERT;    /* 方向校正 */
        right_pwm *= RIGHT_INVERT;
        set_motors(left_pwm, right_pwm);

        /* ── ODOM_DATA @ 50Hz ── */
        if (now - last_odom >= ODOM_INTERVAL_MS)
        {
            last_odom = now;
            uint16_t ts = (uint16_t)(now & 0xFFFF);

            int32_t left_total  = encoder_get_left_total();
            int32_t right_total = encoder_get_right_total();
            float left_speed    = (float)encoder_get_left_speed();
            float right_speed   = (float)encoder_get_right_speed();

            protocol_send_odom_data(
                left_total, right_total,
                left_speed, right_speed,
                0, 0, 0,      /* IMU 未安装 */
                ts);
        }

        /* ── STATUS @ 10Hz ── */
        if (now - last_status >= STATUS_INTERVAL_MS)
        {
            last_status = now;
            uint8_t motor_state  = (cmd_received || target_linear != 0.0f || target_angular != 0.0f) ? 0x03 : 0x00;
            uint8_t sensor_state = 0x00;   /* 急停/碰撞未安装 */
            int16_t mcu_temp     = 2500;   /* 25.00°C 占位 */
            uint16_t error_code  = 0;
            protocol_send_status(motor_state, sensor_state, mcu_temp, error_code);
        }

        /* ── HEARTBEAT @ 1Hz ── */
        if (now - last_heartbeat >= HEARTBEAT_INTERVAL_MS)
        {
            last_heartbeat = now;
            protocol_send_heartbeat();
        }

        bsp_delay_ms(1);
    }
}
