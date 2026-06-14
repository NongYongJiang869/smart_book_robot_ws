#include "bsp_delay.h"

#include "stm32f10x.h"

static uint8_t g_delay_inited;

static uint32_t bsp_delay_tim4_clk_hz(void)
{
  RCC_ClocksTypeDef clocks;
  uint32_t tim_clk;

  RCC_GetClocksFreq(&clocks);
  tim_clk = clocks.PCLK1_Frequency;

  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    tim_clk *= 2U;
  }

  return tim_clk;
}

void bsp_delay_init(void)
{
  TIM_TimeBaseInitTypeDef tim = {0};
  uint32_t tim_clk = bsp_delay_tim4_clk_hz();
  uint16_t prescaler = 0;

  if (tim_clk >= 1000000U)
  {
    prescaler = (uint16_t)((tim_clk / 1000000U) - 1U);
  }

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  tim.TIM_Prescaler = prescaler;
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  tim.TIM_Period = 0xFFFF;
  tim.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &tim);

  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_Cmd(TIM4, ENABLE);

  g_delay_inited = 1U;
}

void bsp_delay_us(uint32_t us)
{
  if (!g_delay_inited)
  {
    bsp_delay_init();
  }

  while (us > 0U)
  {
    uint32_t chunk = (us > 60000U) ? 60000U : us;

    TIM_SetAutoreload(TIM4, (uint16_t)(chunk - 1U));
    TIM_SetCounter(TIM4, 0U);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);

    while (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == RESET)
    {
    }

    us -= chunk;
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
  bsp_delay_us((uint32_t)count);
}