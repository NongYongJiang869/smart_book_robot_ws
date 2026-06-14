/**
 * @file bsp_delay.c
 * @brief SysTick 精确定时 + 累计时间戳
 *
 * 使用 SysTick (Cortex-M3 内核定时器) 替代 TIM4,
 * 将 TIM4 释放给右路电机编码器使用。
 *
 * SysTick 时钟 = HCLK/8 = 9MHz, 9 ticks = 1us
 */

#include "bsp_delay.h"
#include "stm32f10x.h"

static uint8_t  g_delay_inited;
static volatile uint32_t g_total_us;  /* 累计延时微秒数, 用于 bsp_delay_get_ms() */

void bsp_delay_init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    g_total_us = 0U;
    g_delay_inited = 1U;
}

/**
 * @brief 微秒级延时 (最大单次 1864ms)
 */
void bsp_delay_us(uint32_t us)
{
    if (!g_delay_inited)
    {
        bsp_delay_init();
    }

    g_total_us += us;

    /* SysTick 时钟 = 72MHz/8 = 9MHz → ticks = us * 9 */
    uint32_t ticks = us * 9U;

    while (ticks > 0U)
    {
        uint32_t load = (ticks > 0xFFFFFFU) ? 0xFFFFFFU : ticks;

        SysTick->LOAD = load;
        SysTick->VAL  = 0U;
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

        /* 等待 COUNTFLAG 置位 (计数到 0) */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U)
        {
        }

        SysTick->CTRL = 0U;
        ticks -= load;
    }
}

void bsp_delay_ms(uint32_t ms)
{
    while (ms > 0U)
    {
        bsp_delay_us(1000U);
        ms--;
    }
}

void bsp_delay_cycles(volatile uint32_t count)
{
    bsp_delay_us(count);
}

/**
 * @brief 获取系统启动以来累计的毫秒时间戳
 *
 * 基于 bsp_delay_us 的累计微秒计数器, 精度取决于调用频率。
 * 适合在测试循环中判断超时 (如 while(delay_get_ms()-t0 < timeout))。
 */
uint32_t bsp_delay_get_ms(void)
{
    return g_total_us / 1000U;
}
