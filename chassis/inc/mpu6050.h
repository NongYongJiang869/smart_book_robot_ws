/**
 * @file mpu6050.h
 * @brief MPU6050 6轴IMU驱动 — I2C2 (PB10=SCL, PB11=SDA)
 *
 * 量程: 陀螺 ±250°/s, 加速度 ±2g
 * 输出: gyro_z (°/s×1000), accel_x/y (m/s²×1000)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/** @brief 初始化 I2C2 + MPU6050 */
void mpu6050_init(void);

/**
 * @brief 读取传感器数据
 * @param gyro_z  陀螺Z轴 (度/秒 × 1000)
 * @param accel_x 加速度X (m/s² × 1000)
 * @param accel_y 加速度Y (m/s² × 1000)
 * @return 0=成功, 非0=I2C错误
 */
int mpu6050_read(int16_t *gyro_z, int16_t *accel_x, int16_t *accel_y);

/** @brief 返回 1=MPU6050 已检测到并正常工作 */
int mpu6050_is_ok(void);

/**
 * @brief 获取陀螺 Z 轴零偏 (校准值)
 * @return 零偏 (LSB, ±250°/s 量程)
 */
int16_t mpu6050_get_gyro_offset(void);

#endif /* MPU6050_H */
