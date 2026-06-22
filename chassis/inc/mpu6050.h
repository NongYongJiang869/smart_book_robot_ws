/**
 * @file mpu6050.h
 * @brief MPU6050 6轴IMU — 硬件 I2C1 重映射 (PB8=SCL, PB9=SDA)
 *
 * 量程: 陀螺 ±250°/s (LSB=131), 加速度 ±2g (LSB=16384)
 * 输出: 原始 LSB (扣除零偏), ROS2 侧转换为物理单位
 *
 * 校准: 上电静止采样 500 次 (陀螺+加速度双校准) + 运行时陀螺零偏跟踪
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/** @brief 初始化硬件 I2C1 + MPU6050 + 全轴上电校准 */
void mpu6050_init(void);

/**
 * @brief 读取传感器数据 (已扣除校准零偏, 原始LSB)
 * @param gyro_z  陀螺Z轴 (LSB, 131 per °/s), 正值=左转
 * @param accel_x 加速度X (LSB, 16384 per g)
 * @param accel_y 加速度Y (LSB, 16384 per g)
 * @return 0=成功, 非0=错误
 */
int mpu6050_read(int16_t *gyro_z, int16_t *accel_x, int16_t *accel_y);

/** @brief MPU6050 是否正常工作 */
int mpu6050_is_ok(void);

/** @brief 运行时零偏跟踪 — 主循环每次 IMU 读取后调用 */
void mpu6050_track_bias(int is_stationary);

/* ── 校准质量查询 ── */

int16_t mpu6050_get_gyro_offset(void);      /* 陀螺 Z 零偏 (LSB) */
int16_t mpu6050_get_gyro_sigma(void);       /* 陀螺 Z 噪声 (°/s×1000) */
int16_t mpu6050_get_accel_offset_x(void);   /* 加速度 X 零偏 (LSB) */
int16_t mpu6050_get_accel_offset_y(void);   /* 加速度 Y 零偏 (LSB) */
int16_t mpu6050_get_accel_sigma_x(void);    /* 加速度 X 噪声 (m/s²×1000) */
int16_t mpu6050_get_accel_sigma_y(void);    /* 加速度 Y 噪声 (m/s²×1000) */

#endif /* MPU6050_H */
