/**
 * @file protocol.h
 * @brief STM32 ↔ RDK X5 二进制帧协议 (设计文档 03_communication_protocols.md §1)
 *
 * 帧格式: 0x5A 0xA5 | len(1B) | type(1B) | seq(1B) | payload(0~255B) | CRC16(2B,LE)
 * CRC-16-CCITT (poly=0x1021, init=0x0000), 覆盖 type+seq+payload
 * 多字节字段全部 Little-Endian
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* ── 帧类型码 ── */
#define PROTO_TYPE_ODOM_DATA    0x01
#define PROTO_TYPE_STATUS       0x02
#define PROTO_TYPE_ERROR        0x04
#define PROTO_TYPE_ACK          0x05
#define PROTO_TYPE_HEARTBEAT    0x06

/* ── 错误码 ── */
#define PROTO_ERR_ESTOP          0x0001
#define PROTO_ERR_FRONT_COLLISION 0x0002
#define PROTO_ERR_REAR_COLLISION  0x0004
#define PROTO_ERR_COMM_TIMEOUT    0x0100
#define PROTO_ERR_IMU_FAULT       0x0400

/* ── 接收端结构 ── */

/** VEL_CMD 解析结果 */
typedef struct {
    float linear_x;    /* 目标线速度 m/s */
    float angular_z;   /* 目标角速度 rad/s */
    uint8_t seq;       /* 帧序号 */
} VelCmd;

/* ── 发送 API ── */

/**
 * @brief 初始化协议模块 (重置发送序号)
 */
void protocol_init(void);

/**
 * @brief 构建并发送 ODOM_DATA 帧 (0x01)
 *
 * 负载 24B (LE):
 *   [0:4]  int32  left_enc       左轮编码器累计值
 *   [4:8]  int32  right_enc      右轮编码器累计值
 *   [8:12] float  left_wheel_v   左轮瞬时速度 (m/s)
 *   [12:16]float  right_wheel_v  右轮瞬时速度 (m/s)
 *   [16:18]int16  gyro_z         陀螺Z ×1000 (°/s) — 无IMU时填0
 *   [18:20]int16  accel_x        加速度X ×1000 (m/s²) — 无IMU时填0
 *   [20:22]int16  accel_y        加速度Y ×1000 (m/s²) — 无IMU时填0
 *   [22:24]uint16 timestamp_ms   毫秒时间戳 (0~65535循环)
 */
void protocol_send_odom_data(int32_t left_enc, int32_t right_enc,
                             float left_speed, float right_speed,
                             int16_t gyro_z, int16_t accel_x, int16_t accel_y,
                             uint16_t timestamp_ms);

/**
 * @brief 构建并发送 STATUS 帧 (0x02)
 *
 * 负载 6B:
 *   [0] uint8  motor_state   位0~3: 4路电机使能
 *   [1] uint8  sensor_state  位0:急停 位1:前碰 位2:后碰
 *   [2:4]int16  mcu_temp     MCU温度×100 (°C)
 *   [4:6]uint16 error_code   错误码
 */
void protocol_send_status(uint8_t motor_state, uint8_t sensor_state,
                          int16_t mcu_temp, uint16_t error_code);

/**
 * @brief 构建并发送 ERROR 帧 (0x04)
 */
void protocol_send_error(uint16_t error_code);

/**
 * @brief 构建并发送 HEARTBEAT 帧 (0x06, 无负载)
 */
void protocol_send_heartbeat(void);

/* ── 接收 API ── */

/**
 * @brief 从串口 RX 缓冲区读取并解析一帧
 *
 * 在 main 循环中轮询调用:
 *   1. 从 uart_rx_get() 逐字节读取
 *   2. 寻找 0x5A 0xA5 帧头
 *   3. 读长度 → 类型 → 序号 → 负载 → CRC
 *   4. 校验 CRC, 返回帧类型和负载
 *
 * @param out_type    输出帧类型 (0=无帧)
 * @param out_payload 输出负载数据指针 (指向内部 static buffer)
 * @param out_len     输出负载长度
 * @return 1=成功解析一帧, 0=暂无完整帧
 */
int protocol_try_parse(uint8_t *out_type, const uint8_t **out_payload, int *out_len);

/**
 * @brief 解析 VEL_CMD 负载 → VelCmd 结构
 * @return 1=成功, 0=负载长度错误
 */
int protocol_parse_vel_cmd(const uint8_t *payload, int len, VelCmd *cmd);

#endif /* PROTOCOL_H */
