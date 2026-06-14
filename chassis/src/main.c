#include "stm32f10x.h"
#include "motor.h"
#include "bsp_delay.h"

#define TEST_SPEED  600   /* 测试速度 0~999 */
#define TEST_DELAY  2000  /* 每个动作持续 ms */

int main(void)
{
    motor_init();
    bsp_delay_init();

    while (1)
    {
        /* 1. 前进 */
        motor_set(MOTOR_BOTH, MOTOR_FORWARD, TEST_SPEED);
        bsp_delay_ms(TEST_DELAY);

        /* 2. 后退 */
        motor_set(MOTOR_BOTH, MOTOR_BACKWARD, TEST_SPEED);
        bsp_delay_ms(TEST_DELAY);

        /* 3. 左转 (左轮后退, 右轮前进) */
        motor_set(MOTOR_LEFT, MOTOR_BACKWARD, TEST_SPEED);
        motor_set(MOTOR_RIGHT, MOTOR_FORWARD, TEST_SPEED);
        bsp_delay_ms(TEST_DELAY);

        /* 4. 右转 (左轮前进, 右轮后退) */
        motor_set(MOTOR_LEFT, MOTOR_FORWARD, TEST_SPEED);
        motor_set(MOTOR_RIGHT, MOTOR_BACKWARD, TEST_SPEED);
        bsp_delay_ms(TEST_DELAY);

        /* 5. 停止 */
        motor_stop(MOTOR_BOTH);

        /* 停顿3秒后循环 */
        bsp_delay_ms(3000);
    }
}
