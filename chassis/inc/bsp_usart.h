#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>

/* ── 环形缓冲区大小 ── */
#define UART_RX_BUF_SIZE  256

void bsp_usart2_init(void);

/* ── RX 环形缓冲区 (ISR 安全) ── */

/**
 * @brief 向 RX 缓冲区写入一个字节 (在 USART2 ISR 中调用)
 * @return 1=成功, 0=缓冲区满
 */
int uart_rx_put(uint8_t byte);

/**
 * @brief 从 RX 缓冲区读取一个字节 (主循环调用)
 * @return 1=成功, 0=缓冲区空
 */
int uart_rx_get(uint8_t *byte);

/**
 * @brief 查询 RX 缓冲区中的可用字节数
 */
int uart_rx_available(void);

#endif /* BSP_USART_H */
