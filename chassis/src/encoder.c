/**
 * @file encoder.c
 * @brief 左右路电机编码器驱动
 *
 * 左路: TIM1 正交编码器模式  — PA8(CH1) + PA9(CH2)
 * 右路: TIM4 正交编码器模式  — PB6(CH1) + PB7(CH2)
 * 采样: TIM3 定时器 @ 50Hz, 在中断中计算瞬时转速
 *
 * 转速 = 相邻两次 CNT 差值 / 采样周期(0.02s)
 *       正值 = 前进方向, 负值 = 后退方向
 *
 * 硬件:
 *   左路编码器 C1→PA8(TIM1_CH1), C2→PA9(TIM1_CH2)
 *   右路编码器 C1→PB6(TIM4_CH1), C2→PB7(TIM4_CH2)
 *   同侧前后轮编码器并联, 取一路即可
 */

#include "encoder.h"
#include "stm32f10x.h"

/* ================================================================
 * 全局变量 — 在 TIM3 ISR 中更新, 主循环只读
 * ================================================================ */

static volatile int32_t g_left_speed;         /* 左轮瞬时转速 (pulse/s) */
static volatile int32_t g_right_speed;        /* 右轮瞬时转速 (pulse/s) */
static volatile int32_t g_left_total;         /* 左轮累计脉冲 */
static volatile int32_t g_right_total;        /* 右轮累计脉冲 */

static uint16_t g_left_last_cnt;              /* 上一次左路 CNT 值 */
static uint16_t g_right_last_cnt;             /* 上一次右路 CNT 值 */
static uint8_t  g_first_sample;               /* 首次采样标志 */

/* ================================================================
 * 初始化
 * ================================================================ */

void encoder_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_ICInitTypeDef tim_ic;

    /* ---------- 时钟 ---------- */
    /* TIM1: APB2, TIM3+TIM4: APB1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    /* ================================================================
     * TIM1 编码器模式 — 左路电机 (PA8=CH1, PA9=CH2)
     * ================================================================ */
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;   /* 编码器信号: 浮空输入 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Period = 0xFFFF;             /* 16位最大值, 充分利用计数范围 */
    tim_base.TIM_Prescaler = 0;               /* 不分频, 直接对编码器脉冲计数 */
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tim_base);

    /* 正交编码器: TI1+TI2 双沿计数 (4倍频, 最高分辨率) */
    TIM_EncoderInterfaceConfig(TIM1,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising);
    TIM_ICStructInit(&tim_ic);
    TIM_ICInit(TIM1, &tim_ic);
    TIM_Cmd(TIM1, ENABLE);

    /* ================================================================
     * TIM4 编码器模式 — 右路电机 (PB6=CH1, PB7=CH2)
     * ================================================================ */
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Period = 0xFFFF;
    tim_base.TIM_Prescaler = 0;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tim_base);

    TIM_EncoderInterfaceConfig(TIM4,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising);
    TIM_ICStructInit(&tim_ic);
    TIM_ICInit(TIM4, &tim_ic);
    TIM_Cmd(TIM4, ENABLE);

    /* ================================================================
     * TIM3 采样定时器 — 50Hz (20ms) 定期计算转速
     * 72MHz / (71+1) / (19999+1) = 50Hz
     * ================================================================ */
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 71;              /* 72MHz / 72 = 1MHz */
    tim_base.TIM_Period = 19999;              /* 1MHz / 20000 = 50Hz */
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim_base);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); /* 使能更新中断 */
    TIM_Cmd(TIM3, ENABLE);

    /* ---------- NVIC: TIM3 中断优先级 ---------- */
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* ---------- 初始状态 ---------- */
    g_left_speed = 0;
    g_right_speed = 0;
    g_left_total = 0;
    g_right_total = 0;
    g_left_last_cnt = 0;
    g_right_last_cnt = 0;
    g_first_sample = 1;
}

/* ================================================================
 * 周期性更新 (TIM3 ISR 调用, f=50Hz)
 * ================================================================ */

void encoder_update(void)
{
    uint16_t left_cnt, right_cnt;
    int16_t left_delta, right_delta;

    left_cnt  = TIM_GetCounter(TIM1);
    right_cnt = TIM_GetCounter(TIM4);

    if (g_first_sample)
    {
        /* 首次: 仅记录基准值, 不计算转速 */
        g_left_last_cnt  = left_cnt;
        g_right_last_cnt = right_cnt;
        g_first_sample = 0;
        return;
    }

    /* 有符号差值 (自动处理 16bit 回绕, 最大 32767) */
    left_delta  = (int16_t)(left_cnt  - g_left_last_cnt);
    right_delta = (int16_t)(right_cnt - g_right_last_cnt);

    g_left_last_cnt  = left_cnt;
    g_right_last_cnt = right_cnt;

    /* 瞬时转速 = delta / 0.02s = delta * 50 (pulse/s) */
    g_left_speed  = (int32_t)left_delta  * 50;
    g_right_speed = (int32_t)right_delta * 50;

    /* 累计值 (带符号累加, 代表相对位置) */
    g_left_total  += left_delta;
    g_right_total += right_delta;
}

/* ================================================================
 * 查询接口
 * ================================================================ */

int32_t encoder_get_left_speed(void)
{
    return g_left_speed;
}

int32_t encoder_get_right_speed(void)
{
    return g_right_speed;
}

int32_t encoder_get_left_total(void)
{
    return g_left_total;
}

int32_t encoder_get_right_total(void)
{
    return g_right_total;
}
