#include "motor.h"
#include "stm32f10x.h"

/* TB6612 引脚定义
 * AIN1 -> PB3  (左电机方向1)
 * AIN2 -> PA4  (左电机方向2)
 * STBY -> PA12 (待机/使能)
 * BIN1 -> PA5  (右电机方向1)
 * BIN2 -> PA6  (右电机方向2)
 * PWMA -> PA0  (左电机PWM, TIM2_CH1)
 * PWMB -> PA1  (右电机PWM, TIM2_CH2)
 */

/* 控制端口 */
#define MOTOR_AIN1_PORT   GPIOB
#define MOTOR_AIN1_PIN    GPIO_Pin_3

#define MOTOR_AIN2_PORT   GPIOA
#define MOTOR_AIN2_PIN    GPIO_Pin_4

#define MOTOR_STBY_PORT   GPIOA
#define MOTOR_STBY_PIN    GPIO_Pin_12

#define MOTOR_BIN1_PORT   GPIOA
#define MOTOR_BIN1_PIN    GPIO_Pin_5

#define MOTOR_BIN2_PORT   GPIOA
#define MOTOR_BIN2_PIN    GPIO_Pin_6

/* PWM端口 */
#define MOTOR_PWMA_PORT   GPIOA
#define MOTOR_PWMA_PIN    GPIO_Pin_0

#define MOTOR_PWMB_PORT   GPIOA
#define MOTOR_PWMB_PIN    GPIO_Pin_1

/* PWM参数 */
#define MOTOR_PWM_PERIOD  999  /* 1000个计数单位, 0~999 */

void motor_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef tim_oc;

    /* ========== 时钟使能 ========== */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* ========== 控制引脚配置 ========== */
    /* PA4 (AIN2), PA5 (BIN1), PA6 (BIN2), PA12 (STBY) */
    gpio.GPIO_Pin = MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN | MOTOR_STBY_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    /* PB3 (AIN1) */
    gpio.GPIO_Pin = MOTOR_AIN1_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);

    /* 初始状态：全部低电平，STBY拉低禁用 */
    GPIO_ResetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    GPIO_ResetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    GPIO_ResetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    GPIO_ResetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    GPIO_ResetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    /* ========== PWM引脚配置：复用推挽输出 ========== */
    /* PA0 (PWMA / TIM2_CH1) */
    gpio.GPIO_Pin = MOTOR_PWMA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(MOTOR_PWMA_PORT, &gpio);

    /* PA1 (PWMB / TIM2_CH2) */
    gpio.GPIO_Pin = MOTOR_PWMB_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(MOTOR_PWMB_PORT, &gpio);

    /* ========== TIM2 PWM配置 ========== */
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 71;          /* 72MHz / (71+1) = 1MHz → 1us/tick */
    tim_base.TIM_Period = MOTOR_PWM_PERIOD; /* 1000us = 1kHz PWM频率 */
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_base);

    /* CH1 (PWMA) PWM模式1 */
    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse = 0;  /* 初始占空比0 */
    TIM_OC1Init(TIM2, &tim_oc);

    /* CH2 (PWMB) PWM模式1 */
    tim_oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM2, &tim_oc);

    /* 预装载使能 */
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* 自动重装载预装载使能 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* 启动TIM2 */
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief 设置电机方向和速度
 * @param ch   电机通道 (MOTOR_LEFT / MOTOR_RIGHT / MOTOR_BOTH)
 * @param dir  方向 (MOTOR_FORWARD / MOTOR_BACKWARD / MOTOR_STOP)
 * @param speed 速度 0~999 (占空比, 0=停转, 999=全速)
 */
void motor_set(MotorChannel ch, MotorDir dir, uint16_t speed)
{
    if (speed > MOTOR_PWM_PERIOD)
    {
        speed = MOTOR_PWM_PERIOD;
    }

    /* 先使能TB6612 */
    GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);

    if (ch == MOTOR_LEFT || ch == MOTOR_BOTH)
    {
        switch (dir)
        {
        case MOTOR_FORWARD:
            GPIO_SetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
            GPIO_ResetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
            break;
        case MOTOR_BACKWARD:
            GPIO_ResetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
            GPIO_SetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
            break;
        case MOTOR_STOP:
        default:
            GPIO_ResetBits(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
            GPIO_ResetBits(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
            speed = 0;
            break;
        }
        TIM_SetCompare1(TIM2, speed);
    }

    if (ch == MOTOR_RIGHT || ch == MOTOR_BOTH)
    {
        switch (dir)
        {
        case MOTOR_FORWARD:
            GPIO_SetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
            GPIO_ResetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
            break;
        case MOTOR_BACKWARD:
            GPIO_ResetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
            GPIO_SetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
            break;
        case MOTOR_STOP:
        default:
            GPIO_ResetBits(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
            GPIO_ResetBits(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
            speed = 0;
            break;
        }
        TIM_SetCompare2(TIM2, speed);
    }
}

/**
 * @brief 停止指定通道电机
 */
void motor_stop(MotorChannel ch)
{
    motor_set(ch, MOTOR_STOP, 0);

    /* 如果两个电机都停了，拉低STBY省电 */
    if (ch == MOTOR_BOTH)
    {
        GPIO_ResetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
    }
}
