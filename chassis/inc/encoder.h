#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/**
 * @brief 编码器模块初始化
 *
 * 左路: TIM1 编码器模式, PA8=CH1, PA9=CH2
 * 右路: TIM4 编码器模式, PB6=CH1, PB7=CH2
 * 采样: TIM3 50Hz 定时触发, 在中断中计算转速
 */
void encoder_init(void);

/**
 * @brief 获取左路电机转速 (编码器脉冲/秒)
 * @return 正值=前进, 负值=后退
 */
int32_t encoder_get_left_speed(void);

/**
 * @brief 获取右路电机转速 (编码器脉冲/秒)
 * @return 正值=前进, 负值=后退
 */
int32_t encoder_get_right_speed(void);

/**
 * @brief 获取左路编码器累计值 (圈数*CPR, 不清零)
 */
int32_t encoder_get_left_total(void);

/**
 * @brief 获取右路编码器累计值 (圈数*CPR, 不清零)
 */
int32_t encoder_get_right_total(void);

/**
 * @brief 编码器周期性更新 (在 TIM3 ISR 中调用, 50Hz)
 *        计算瞬时转速并更新累计值
 */
void encoder_update(void);

#endif /* ENCODER_H */
