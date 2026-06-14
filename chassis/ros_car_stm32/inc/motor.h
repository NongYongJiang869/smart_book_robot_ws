#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 电机方向 */
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
} MotorDir;

/* 电机通道 */
typedef enum {
    MOTOR_LEFT  = 0,  /* A通道 - 左路电机（两个，一前一后） */
    MOTOR_RIGHT = 1,  /* B通道 - 右路电机（两个，一前一后） */
    MOTOR_BOTH  = 2,
} MotorChannel;

void motor_init(void);
void motor_set(MotorChannel ch, MotorDir dir, uint16_t speed);
void motor_stop(MotorChannel ch);

#endif /* MOTOR_H */
