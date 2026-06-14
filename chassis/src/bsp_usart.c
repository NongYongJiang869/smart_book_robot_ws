#include "bsp_usart.h"

#include "stm32f10x.h"

void bsp_usart1_init(void)
{
  GPIO_InitTypeDef gpio = {0};
  USART_InitTypeDef usart = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  usart.USART_BaudRate = 115200;
  usart.USART_WordLength = USART_WordLength_8b;
  usart.USART_StopBits = USART_StopBits_1;
  usart.USART_Parity = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

  USART_Init(USART1, &usart);
  USART_Cmd(USART1, ENABLE);
}

int _write(int file, char *ptr, int len)
{
  int i;
  (void)file;

  for (i = 0; i < len; i++)
  {
    USART_SendData(USART1, (uint8_t)ptr[i]);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
  }

  return len;
}
