#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include <stdint.h>

void bsp_delay_init(void);
void bsp_delay_us(uint32_t us);
void bsp_delay_ms(uint32_t ms);
void bsp_delay_cycles(volatile uint32_t count);
uint32_t bsp_delay_get_ms(void);

#endif /* BSP_DELAY_H */