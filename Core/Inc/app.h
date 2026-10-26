#ifndef APP_H
#define APP_H

#include "main.h"

typedef struct {
    I2C_HandleTypeDef *oled_i2c;
    UART_HandleTypeDef *sensor_uart;
    UART_HandleTypeDef *console_uart;
    TIM_HandleTypeDef *motor1_pwm;
    TIM_HandleTypeDef *motor2_pwm;
    void (*i2c1_reinit)(void);
} AppHardware;

void App_Init(const AppHardware *hardware);
void App_RunOnce(void);

#endif /* APP_H */
