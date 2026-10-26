#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

/* 采集周期：每秒保存一个采样点。 */
#define COUNTDOWN_TIME_MS_A     120000U
#define COUNTDOWN_POINTS_A      (COUNTDOWN_TIME_MS_A / 1000U)
#define COUNTDOWN_TIME_MS_B     20000U
#define COUNTDOWN_POINTS_B      (COUNTDOWN_TIME_MS_B / 1000U)

/* 两次泵液均为 5 秒，中间静置 1 秒。 */
#define PUMP_RUN_TIME_MS        5000U
#define SETTLE_TIME_MS          1000U
#define SENSOR_SAMPLE_TIME_MS   1000U

/* 电机 1：TB6612，相位引脚位于 GPIOB。 */
#define MOTOR1_AIN1_PORT        GPIOB
#define MOTOR1_AIN1_PIN         GPIO_PIN_12
#define MOTOR1_AIN2_PORT        GPIOB
#define MOTOR1_AIN2_PIN         GPIO_PIN_13
#define MOTOR1_BIN1_PORT        GPIOB
#define MOTOR1_BIN1_PIN         GPIO_PIN_14
#define MOTOR1_BIN2_PORT        GPIOB
#define MOTOR1_BIN2_PIN         GPIO_PIN_15

/* 电机 2：TB6612。 */
#define MOTOR2_AIN1_PORT        GPIOB
#define MOTOR2_AIN1_PIN         GPIO_PIN_0
#define MOTOR2_AIN2_PORT        GPIOB
#define MOTOR2_AIN2_PIN         GPIO_PIN_1
#define MOTOR2_BIN1_PORT        GPIOA
#define MOTOR2_BIN1_PIN         GPIO_PIN_1
#define MOTOR2_BIN2_PORT        GPIOA
#define MOTOR2_BIN2_PIN         GPIO_PIN_4

/* 当前工程实际在 PA5 拉高驱动待机脚。 */
#define MOTOR_STBY_PORT         GPIOA
#define MOTOR_STBY_PIN          GPIO_PIN_5
#define MOTOR_STEP_DELAY_MS     1U
#define MOTOR_PWM_DUTY          500U
#define MOTOR1_TARGET_STEPS     551U
#define MOTOR2_TARGET_STEPS     2204U

/* A4988：EN 低电平有效，位置采用开环相对脉冲计数。 */
#define SLIDE_STEP_PORT         GPIOA
#define SLIDE_STEP_PIN          GPIO_PIN_7
#define SLIDE_DIR_PORT          GPIOB
#define SLIDE_DIR_PIN           GPIO_PIN_10
#define SLIDE_EN_PORT           GPIOB
#define SLIDE_EN_PIN            GPIO_PIN_11
#define SLIDE_SHIFT_STEPS       105
#define SLIDE_STEP_DELAY_MS     1U
#define SLIDE_IDLE_OFF_MS       5000U

#define KEY2_PORT               GPIOA
#define KEY2_PIN                GPIO_PIN_11
#define KEY_DEBOUNCE_MS         20U

#define LED_PORT                GPIOA
#define LED_PIN                 GPIO_PIN_0
#define PUMP_PORT               GPIOB
#define PUMP_PIN                GPIO_PIN_4

/* PB5 用于 RS485 半双工方向控制：高发送，低接收。 */
#define RS485_CTRL_PORT         GPIOB
#define RS485_CTRL_PIN          GPIO_PIN_5

#define CONSOLE_BUFFER_SIZE     16U
#define CONSOLE_IDLE_TIME_MS    50U
#define CONSOLE_MOTOR_MAX_STEPS 999999U
#define CONSOLE_SLIDE_MAX_STEPS 9999U

/* 标定关系为 y = kx + b，x 的单位为 ppm。 */
#define CAL_A_K                 15.59213f
#define CAL_A_B                 (-19.48466f)
#define CAL_A_MIN_PPM           3.0f
#define CAL_A_MAX_PPM           18.0f
#define CAL_B_K                 2.49739f
#define CAL_B_B                 96.09863f
#define CAL_B_MIN_PPM           15.0f
#define CAL_B_MAX_PPM           65.0f

#endif /* APP_CONFIG_H */
