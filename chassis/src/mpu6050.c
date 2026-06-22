/**
 * @file mpu6050.c
 * @brief MPU6050 6轴IMU — 软件 GPIO 模拟 I2C (PB8=SCL, PB9=SDA) + 全轴校准
 *
 * 为什么用软件 I2C 而不是硬件 I2C:
 *   - 硬件 I2C1 remap (PB8/PB9) 在没有外部上拉电阻时不可靠
 *   - 软件 GPIO 可以灵活控制时序, 内部上拉 40kΩ 也能工作 (降速即可)
 *
 * 校准策略:
 *   1. 上电静止校准 — 采样 500 次, 3σ 剔除, 陀螺+加速度双校准
 *   2. 运行时陀螺零偏跟踪 — 小车静止时自动修正 (应对温度漂移)
 */

#include "mpu6050.h"
#include "stm32f10x.h"
#include "bsp_delay.h"
#include <math.h>

/* ── 引脚 ── */
#define I2C_PORT       GPIOB
#define I2C_SCL_PIN    GPIO_Pin_8
#define I2C_SDA_PIN    GPIO_Pin_9

/* ── 软件 I2C 时序 (100kHz @ 72MHz) ── */
#define I2C_HALF_CYCLE_US  5U     /* SCL 半周期 (低/高各 5µs, ~100kHz) */

/* ── MPU6050 ── */
#define MPU_ADDR      0x68
#define REG_PWR_MGMT  0x6B
#define REG_GYRO_CFG  0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_WHO_AM_I  0x75
#define REG_DATA      0x3B

/* ── 上电校准 ── */
#define CALIB_SAMPLES        500
#define CALIB_OUTLIER_SIGMA  3.0f

/* ── 运行时陀螺零偏跟踪 ── */
#define BIAS_WINDOW     50

/* ── 状态 ── */
static int     g_mpu_ok;

/* 陀螺 */
static int16_t g_gyro_offset;        /* Z 轴零偏 (LSB) */
static int16_t g_gyro_sigma;         /* 校准噪声 (°/s×1000) */
static int16_t g_gyro_residual;      /* 上次 offset 修正后的残差 (LSB) */

/* 加速度计 */
static int16_t g_accel_offset_x;     /* X 轴零偏 (LSB) */
static int16_t g_accel_offset_y;     /* Y 轴零偏 (LSB) */
static int16_t g_accel_sigma_x;      /* X 轴校准噪声 (m/s²×1000) */
static int16_t g_accel_sigma_y;      /* Y 轴校准噪声 (m/s²×1000) */

/* 运行时偏置跟踪 */
static float   g_bias_accum;
static int     g_bias_count;

/* ================================================================
 * 软件 I2C 底层 (GPIO 位操作)
 * ================================================================ */

static inline void scl_h(void) { GPIO_SetBits(  I2C_PORT, I2C_SCL_PIN); }
static inline void scl_l(void) { GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN); }
static inline void sda_h(void) { GPIO_SetBits(  I2C_PORT, I2C_SDA_PIN); }
static inline void sda_l(void) { GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN); }
static inline uint8_t sda_read(void) { return GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN); }

static void i2c_delay(void)
{
    bsp_delay_us(I2C_HALF_CYCLE_US);
}

/**
 * @brief 初始化软件 I2C GPIO (PB8=SCL, PB9=SDA, 开漏输出)
 *
 * 两个引脚都配置为开漏输出:
 *   写 1 → 引脚浮空, 由 (内部/外部) 上拉电阻拉到高电平
 *   写 0 → 引脚拉到 GND
 * 读 SDA 用 GPIO_ReadInputDataBit — 即使开漏输出模式也能读到实际电平
 */
static void i2c_sw_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;  /* 开漏 — 匹配 I2C 电气特性 */
    GPIO_Init(I2C_PORT, &gpio);

    /* 初始状态: 总线空闲 (SCL=H, SDA=H) */
    scl_h();
    sda_h();
    bsp_delay_us(100);  /* 等总线稳定 */
}

static void i2c_start(void)
{
    sda_h();
    i2c_delay();
    scl_h();
    i2c_delay();
    sda_l();             /* SCL=H 时 SDA H→L */
    i2c_delay();
    scl_l();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_l();
    i2c_delay();
    scl_h();
    i2c_delay();
    sda_h();             /* SCL=H 时 SDA L→H */
    i2c_delay();
}

/**
 * @brief 发送一个字节, 返回 ACK (0=ACK, 1=NACK)
 */
static int i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--)
    {
        if (byte & (1 << i))
            sda_h();
        else
            sda_l();
        i2c_delay();
        scl_h();          /* SCL H: 接收方采样 SDA */
        i2c_delay();
        scl_l();
    }

    /* 释放 SDA, 读 ACK */
    sda_h();
    i2c_delay();
    scl_h();
    i2c_delay();
    int ack = sda_read(); /* 0=ACK, 1=NACK */
    scl_l();
    i2c_delay();

    return ack;
}

/**
 * @brief 读取一个字节, ack=0 回复 ACK, ack=1 回复 NACK
 */
static uint8_t i2c_read_byte(int ack)
{
    uint8_t byte = 0;

    sda_h();  /* 释放 SDA 让从机驱动 */

    for (int i = 7; i >= 0; i--)
    {
        i2c_delay();
        scl_h();
        i2c_delay();
        if (sda_read())
            byte |= (1 << i);
        scl_l();
    }

    /* 回复 ACK / NACK */
    if (ack)
        sda_h();  /* NACK: 保持高 */
    else
        sda_l();  /* ACK: 拉低 */
    i2c_delay();
    scl_h();
    i2c_delay();
    scl_l();
    i2c_delay();
    sda_h();  /* 释放 */

    return byte;
}

/* ================================================================
 * MPU6050 寄存器读写 (基于软件 I2C)
 * ================================================================ */

static int mpu_write_reg(uint8_t reg, uint8_t data)
{
    i2c_start();
    int ack1 = i2c_write_byte(MPU_ADDR << 1);        /* 地址 + W */
    int ack2 = i2c_write_byte(reg);                   /* 寄存器号 */
    int ack3 = i2c_write_byte(data);                  /* 数据 */
    i2c_stop();
    return (ack1 == 0 && ack2 == 0 && ack3 == 0) ? 0 : -1;
}

static int mpu_read_regs(uint8_t reg, uint8_t *buf, int len)
{
    if (len <= 0) return -1;

    /* 先写寄存器地址 */
    i2c_start();
    int ack1 = i2c_write_byte(MPU_ADDR << 1);        /* 地址 + W */
    if (ack1) { i2c_stop(); return -1; }
    int ack2 = i2c_write_byte(reg);                   /* 寄存器号 */
    if (ack2) { i2c_stop(); return -1; }

    /* 重复起始 + 读 */
    i2c_start();
    int ack3 = i2c_write_byte((MPU_ADDR << 1) | 1);  /* 地址 + R */
    if (ack3) { i2c_stop(); return -1; }

    for (int i = 0; i < len; i++)
    {
        buf[i] = i2c_read_byte(i == len - 1);         /* 最后一字节回复 NACK */
    }
    i2c_stop();
    return 0;
}

/* ================================================================
 * 校准辅助: 对一组样本做 3σ 剔除 → 返回过滤后的均值
 * ================================================================ */

/**
 * @brief 对 int16_t 样本数组做 3σ 剔除后取均值
 */
static void calibrate_channel(const int16_t *samples, int n,
                              int16_t *offset_out, int16_t *sigma_out)
{
    if (n < 20) return;

    /* 均值 */
    float sum = 0.0f, sum_sq = 0.0f;
    for (int i = 0; i < n; i++) {
        sum   += (float)samples[i];
        sum_sq += (float)samples[i] * (float)samples[i];
    }
    float mean = sum / n;
    float var  = sum_sq / n - mean * mean;
    if (var < 0) var = 0;
    float sigma = sqrtf(var);

    if (sigma_out) *sigma_out = (int16_t)sigma;

    /* 3σ 剔除 + 重算均值 */
    float lo = mean - CALIB_OUTLIER_SIGMA * sigma;
    float hi = mean + CALIB_OUTLIER_SIGMA * sigma;
    float sum_filt = 0.0f;
    int   n_filt   = 0;

    for (int i = 0; i < n; i++) {
        float v = (float)samples[i];
        if (v >= lo && v <= hi) {
            sum_filt += v;
            n_filt++;
        }
    }

    *offset_out = (n_filt > 10) ? (int16_t)(sum_filt / n_filt)
                                : (int16_t)mean;
}

/* ================================================================
 * 初始化 + 上电校准 (陀螺 + 加速度)
 *
 * 流程: 软件 I2C 初始化 → 唤醒 MPU6050 → 读 WHO_AM_I 验证
 *       → 配置量程 → 500 次静止采样 → 3σ 校准
 * ================================================================ */

void mpu6050_init(void)
{
    g_mpu_ok         = 0;
    g_gyro_offset    = 0;
    g_gyro_sigma     = 0;
    g_gyro_residual  = 0;
    g_accel_offset_x = 0;
    g_accel_offset_y = 0;
    g_accel_sigma_x  = 0;
    g_accel_sigma_y  = 0;
    g_bias_accum     = 0.0f;
    g_bias_count     = 0;

    /* 1. 软件 I2C GPIO 初始化 */
    i2c_sw_init();

    /* 2. 唤醒 MPU6050 (退出睡眠模式) */
    bsp_delay_us(30000);  /* 等 MPU6050 上电稳定 30ms */
    if (mpu_write_reg(REG_PWR_MGMT, 0x00) != 0) return;
    bsp_delay_us(10000);  /* 等时钟稳定 10ms */

    /* 3. WHO_AM_I 验证 */
    uint8_t who;
    if (mpu_read_regs(REG_WHO_AM_I, &who, 1) != 0) return;
    if (who != 0x68) return;

    /* 4. 配置量程: 陀螺 ±250°/s, 加速度 ±2g */
    mpu_write_reg(REG_GYRO_CFG,  0x00);
    mpu_write_reg(REG_ACCEL_CFG, 0x00);

    /* ═══════════════════════════════════════════════════════════
     * 5. 上电校准: 一次性采集 3 通道 (gyro_z, accel_x, accel_y)
     * ═══════════════════════════════════════════════════════════ */

    int16_t gyro_samples[CALIB_SAMPLES];
    int16_t accx_samples[CALIB_SAMPLES];
    int16_t accy_samples[CALIB_SAMPLES];
    int     n = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        bsp_delay_us(3000);  /* 3ms 采样间隔 */

        uint8_t buf[14];
        if (mpu_read_regs(REG_DATA, buf, 14) != 0)
            continue;

        gyro_samples[n] = ((int16_t)buf[12] << 8) | buf[13];
        accx_samples[n] = ((int16_t)buf[0]  << 8) | buf[1];
        accy_samples[n] = ((int16_t)buf[2]  << 8) | buf[3];
        n++;
    }

    if (n < 50) return;

    /* 陀螺 Z 校准 */
    calibrate_channel(gyro_samples, n, &g_gyro_offset, &g_gyro_sigma);
    /* σ 转换: LSB → °/s×1000 */
    g_gyro_sigma = (int16_t)((int32_t)g_gyro_sigma * 1000 / 131);

    /* 加速度 X/Y 校准 */
    int16_t sigma_lsb;
    calibrate_channel(accx_samples, n, &g_accel_offset_x, &sigma_lsb);
    g_accel_sigma_x = (int16_t)((int32_t)sigma_lsb * 9807 / 16384);
    calibrate_channel(accy_samples, n, &g_accel_offset_y, &sigma_lsb);
    g_accel_sigma_y = (int16_t)((int32_t)sigma_lsb * 9807 / 16384);

    g_mpu_ok = 1;
}

/* ================================================================
 * 读取
 * ================================================================ */

int mpu6050_read(int16_t *gyro_z, int16_t *accel_x, int16_t *accel_y)
{
    uint8_t buf[14];
    if (!g_mpu_ok) return -1;

    if (mpu_read_regs(REG_DATA, buf, 14) != 0) {
        g_mpu_ok = 0;
        return -1;
    }

    int16_t raw_ax = ((int16_t)buf[0]  << 8) | buf[1];
    int16_t raw_ay = ((int16_t)buf[2]  << 8) | buf[3];
    int16_t raw_gz = ((int16_t)buf[12] << 8) | buf[13];

    /* 扣除零偏 */
    raw_ax -= g_accel_offset_x;
    raw_ay -= g_accel_offset_y;
    raw_gz -= g_gyro_offset;

    /* 保存陀螺残差 (用于运行时跟踪) */
    g_gyro_residual = raw_gz;

    /* 直接返回原始 LSB (扣除零偏后)
     * 陀螺   ±250°/s: LSB=131 → ROS2 侧 /131.0 得 °/s
     * 加速度 ±2g:     LSB=16384 → ROS2 侧 /16384.0*9.807 得 m/s²
     *
     * 注意: 不在这里做量纲转换, 避免 int16 溢出
     * (250°/s × 1000 = 250000 远超 int16 范围)
     */
    *gyro_z  = raw_gz;
    *accel_x = raw_ax;
    *accel_y = raw_ay;

    return 0;
}

/* ================================================================
 * 运行时陀螺零偏跟踪
 * ================================================================ */

void mpu6050_track_bias(int is_stationary)
{
    if (!g_mpu_ok) return;

    if (!is_stationary) {
        g_bias_accum = 0.0f;
        g_bias_count = 0;
        return;
    }

    g_bias_accum += (float)g_gyro_residual;
    g_bias_count++;

    if (g_bias_count >= BIAS_WINDOW) {
        int16_t delta = (int16_t)(g_bias_accum / (float)g_bias_count);
        if (delta >  100) delta =  100;
        if (delta < -100) delta = -100;
        g_gyro_offset += delta;
        g_bias_accum = 0.0f;
        g_bias_count = 0;
    }
}

/* ================================================================
 * 查询
 * ================================================================ */

int     mpu6050_is_ok(void)             { return g_mpu_ok; }
int16_t mpu6050_get_gyro_offset(void)   { return g_gyro_offset; }
int16_t mpu6050_get_gyro_sigma(void)    { return g_gyro_sigma; }
int16_t mpu6050_get_accel_offset_x(void){ return g_accel_offset_x; }
int16_t mpu6050_get_accel_offset_y(void){ return g_accel_offset_y; }
int16_t mpu6050_get_accel_sigma_x(void) { return g_accel_sigma_x; }
int16_t mpu6050_get_accel_sigma_y(void) { return g_accel_sigma_y; }
