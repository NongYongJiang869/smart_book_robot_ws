/**
 * @file mpu6050.c
 * @brief MPU6050 6轴IMU — 软件 I2C (PB8=SCL, PB9=SDA) + 陀螺校准
 *
 * 使用 GPIO 模拟 I2C (~50kHz), 不依赖硬件 I2C 外设.
 * 上电时自动采样 80 次计算陀螺零偏.
 */

#include "mpu6050.h"
#include "stm32f10x.h"

/* ── 引脚 ── */
#define I2C_PORT    GPIOB
#define I2C_SCL_PIN GPIO_Pin_8
#define I2C_SDA_PIN GPIO_Pin_9

/* ── MPU6050 ── */
#define MPU_ADDR     0x68
#define REG_PWR_MGMT 0x6B
#define REG_GYRO_CFG 0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_WHO_AM_I 0x75
#define REG_DATA     0x3B

/* ── 时序 (72MHz, ~50kHz) ── */
#define I2C_DELAY  720

#define SCL_H() GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define SCL_L() GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define SDA_H() GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_L() GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_IN() GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)

/* ── 状态 ── */
static int g_mpu_ok;
static int16_t g_gyro_offset;

/* ================================================================
 * GPIO + 软件 I2C
 * ================================================================ */

static void i2c_delay(void)
{
    volatile int i;
    for (i = 0; i < I2C_DELAY; i++) {}
}

static void i2c_gpio_init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C_PORT, &gpio);
    SCL_H(); SDA_H();
    for (volatile int i = 0; i < 10000; i++) {}
}

static void i2c_start(void)
{
    SDA_H(); i2c_delay(); SCL_H(); i2c_delay();
    SDA_L(); i2c_delay(); SCL_L(); i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L(); i2c_delay(); SCL_H(); i2c_delay(); SDA_H(); i2c_delay();
}

static int i2c_write_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--)
    {
        if (data & (1 << i)) SDA_H(); else SDA_L();
        i2c_delay(); SCL_H(); i2c_delay(); SCL_L(); i2c_delay();
    }
    SDA_H(); i2c_delay(); SCL_H(); i2c_delay();
    int ack = SDA_IN();
    SCL_L(); i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(int send_ack)
{
    uint8_t data = 0;
    SDA_H();
    for (int i = 7; i >= 0; i--)
    {
        SCL_H(); i2c_delay();
        if (SDA_IN()) data |= (1 << i);
        SCL_L(); i2c_delay();
    }
    if (send_ack) SDA_L(); else SDA_H();
    i2c_delay(); SCL_H(); i2c_delay(); SCL_L(); i2c_delay();
    return data;
}

static void mpu_write_reg(uint8_t reg, uint8_t data)
{
    i2c_start();
    i2c_write_byte(MPU_ADDR << 1);
    i2c_write_byte(reg);
    i2c_write_byte(data);
    i2c_stop();
}

static void mpu_read_regs(uint8_t reg, uint8_t *buf, int len)
{
    i2c_start();
    i2c_write_byte(MPU_ADDR << 1);
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte((MPU_ADDR << 1) | 1);
    for (int i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len - 1);
    i2c_stop();
}

/* ================================================================
 * 初始化 + 校准
 * ================================================================ */

void mpu6050_init(void)
{
    g_mpu_ok = 0;
    g_gyro_offset = 0;

    i2c_gpio_init();
    for (volatile int i = 0; i < 500000; i++) {}

    mpu_write_reg(REG_PWR_MGMT, 0x00);
    for (volatile int i = 0; i < 400000; i++) {}

    uint8_t who;
    mpu_read_regs(REG_WHO_AM_I, &who, 1);
    if (who != 0x68) return;

    mpu_write_reg(REG_GYRO_CFG, 0x00);
    mpu_write_reg(REG_ACCEL_CFG, 0x00);

    /* ── 陀螺校准: 静止采样 80 次 ── */
    int32_t sum = 0; int n = 0;
    uint8_t buf[14];
    for (int i = 0; i < 80; i++)
    {
        for (volatile int d = 0; d < 200000; d++) {}
        mpu_read_regs(REG_DATA, buf, 14);
        sum += ((int16_t)buf[12] << 8) | buf[13];
        n++;
    }
    if (n > 10)
        g_gyro_offset = (int16_t)(sum / n);

    g_mpu_ok = 1;
}

/* ================================================================
 * 读取
 * ================================================================ */

int mpu6050_read(int16_t *gyro_z, int16_t *accel_x, int16_t *accel_y)
{
    uint8_t buf[14];
    if (!g_mpu_ok) return -1;

    mpu_read_regs(REG_DATA, buf, 14);

    int16_t raw_ax = ((int16_t)buf[0] << 8) | buf[1];
    int16_t raw_ay = ((int16_t)buf[2] << 8) | buf[3];
    int16_t raw_gz = ((int16_t)buf[12]<< 8) | buf[13];

    raw_gz -= g_gyro_offset;

    *gyro_z  = (int16_t)((int32_t)raw_gz * 1000 / 131);
    *accel_x = (int16_t)((int32_t)raw_ax * 9807 / 16384);
    *accel_y = (int16_t)((int32_t)raw_ay * 9807 / 16384);

    return 0;
}

int mpu6050_is_ok(void) { return g_mpu_ok; }
int16_t mpu6050_get_gyro_offset(void) { return g_gyro_offset; }
