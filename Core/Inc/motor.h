#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef enum {
    MOTOR_EVENT_NONE = 0,
    MOTOR_EVENT_AUTO_DONE,
    MOTOR_EVENT_MANUAL1_DONE,
    MOTOR_EVENT_MANUAL2_DONE
} MotorEvent;

void Motor_Init(TIM_HandleTypeDef *motor1_pwm, TIM_HandleTypeDef *motor2_pwm);
void Motor_StartAutomatic(uint8_t motor, uint32_t target_steps);
void Motor_StartManual(uint8_t motor, int8_t direction, uint32_t target_steps);
void Motor_Stop(uint8_t motor);
MotorEvent Motor_RunOnce(void);
uint8_t Motor_IsBusy(void);

#endif /* MOTOR_H */
