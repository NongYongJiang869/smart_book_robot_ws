/**
 * @file bsp_usart.c
 * @brief USART2 初始化 + RX 环形缓冲区 (PA2=TX, PA3=RX, 115200 8N1)
 *
 * TX: 阻塞发送 (用于 _write() 和协议帧发送)
 * RX: 中断接收 → 环形缓冲区 (主循环轮询读取)
 */

#include "bsp_usart.h"
#include "stm32f10x.h"

/* ── RX 环形缓冲区 ── */
static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile int rx_head = 0;  /* ISR 写入位置 */
static volatile int rx_tail = 0;  /* 主循环读取位置 */

int uart_rx_put(uint8_t byte)
{
    int next = (rx_head + 1) % UART_RX_BUF_SIZE;
    if (next == rx_tail) return 0;  /* 缓冲区满 */
    rx_buf[rx_head] = byte;
    rx_head = next;
    return 1;
}

int uart_rx_get(uint8_t *byte)
{
    if (rx_head == rx_tail) return 0;  /* 缓冲区空 */
    *byte = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

int uart_rx_available(void)
{
    return (rx_head - rx_tail + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE;
}

/* ── USART2 初始化 ── */

void bsp_usart2_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};
    NVIC_InitTypeDef nvic = {0};

    /* 时钟: GPIOA (APB2), USART2 (APB1) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 = USART2_TX (复用推挽) */
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA3 = USART2_RX (浮空输入) */
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &usart);

    /* 使能 RX 中断 (RXNE: 接收到一个字节) */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* NVIC */
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

/* ── printf 重定向 (TX 阻塞发送) ── */

int _write(int file, char *ptr, int len)
{
    int i;
    (void)file;
    for (i = 0; i < len; i++)
    {
        USART_SendData(USART2, (uint8_t)ptr[i]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
    }
    return len;
}
