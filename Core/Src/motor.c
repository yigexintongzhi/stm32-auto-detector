#include "motor.h"
#include "app_config.h"

typedef struct {
    TIM_HandleTypeDef *motor1_pwm;
    TIM_HandleTypeDef *motor2_pwm;
    uint8_t current_motor;
    uint8_t step_index;
    uint32_t steps_done;
    uint32_t target_steps;
    uint8_t busy;
    uint8_t manual_mode;
    uint8_t manual_motor;
    int8_t manual_dir;
    uint32_t manual_target;
} MotorContext;

static MotorContext motor;

/* 四拍励磁表，每次主循环最多输出一拍。 */
static const uint8_t step_sequence[4][4] = {
    {1, 0, 1, 0},
    {1, 0, 0, 1},
    {0, 1, 0, 1},
    {0, 1, 1, 0}
};

static void Motor_SetPwm(uint8_t motor_number, uint32_t compare)
{
    if (motor_number == 1U) {
        __HAL_TIM_SET_COMPARE(motor.motor1_pwm, TIM_CHANNEL_1, compare);
    } else {
        __HAL_TIM_SET_COMPARE(motor.motor2_pwm, TIM_CHANNEL_1, compare);
    }
}

static void Motor_WritePhase(uint8_t motor_number, uint8_t phase)
{
    if (motor_number == 1U) {
        HAL_GPIO_WritePin(MOTOR1_AIN1_PORT, MOTOR1_AIN1_PIN,
                          step_sequence[phase][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR1_AIN2_PORT, MOTOR1_AIN2_PIN,
                          step_sequence[phase][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR1_BIN1_PORT, MOTOR1_BIN1_PIN,
                          step_sequence[phase][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR1_BIN2_PORT, MOTOR1_BIN2_PIN,
                          step_sequence[phase][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN,
                          step_sequence[phase][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN,
                          step_sequence[phase][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN,
                          step_sequence[phase][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN,
                          step_sequence[phase][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void Motor_ClearPhase(uint8_t motor_number)
{
    if (motor_number == 1U) {
        HAL_GPIO_WritePin(GPIOB,
                          MOTOR1_AIN1_PIN | MOTOR1_AIN2_PIN |
                          MOTOR1_BIN1_PIN | MOTOR1_BIN2_PIN,
                          GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, GPIO_PIN_RESET);
    }
}

static void Motor_DisableOutput(uint8_t motor_number)
{
    /* 先撤 PWM，再清相位引脚。 */
    Motor_SetPwm(motor_number, 0U);
    Motor_ClearPhase(motor_number);
}

void Motor_Init(TIM_HandleTypeDef *motor1_pwm, TIM_HandleTypeDef *motor2_pwm)
{
    motor.motor1_pwm = motor1_pwm;
    motor.motor2_pwm = motor2_pwm;
    motor.current_motor = 1U;
    motor.step_index = 0U;
    motor.steps_done = 0U;
    motor.target_steps = 0U;
    motor.busy = 0U;
    motor.manual_mode = 0U;
    motor.manual_motor = 1U;
    motor.manual_dir = 1;
    motor.manual_target = 0U;

    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(motor.motor1_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(motor.motor2_pwm, TIM_CHANNEL_1);
    Motor_SetPwm(2U, 0U);
    Motor_SetPwm(1U, 0U);
}

void Motor_StartAutomatic(uint8_t motor_number, uint32_t target_steps)
{
    motor.current_motor = motor_number;
    motor.target_steps = target_steps;
    motor.busy = 1U;
    motor.steps_done = 0U;
    motor.step_index = 0U;
    motor.manual_mode = 0U;
    Motor_SetPwm(motor_number, MOTOR_PWM_DUTY);
}

void Motor_StartManual(uint8_t motor_number, int8_t direction, uint32_t target_steps)
{
    motor.manual_mode = 1U;
    motor.manual_motor = motor_number;
    motor.manual_dir = direction;
    motor.manual_target = target_steps;
    motor.current_motor = motor_number;
    motor.busy = 1U;
    motor.steps_done = 0U;
    motor.step_index = 0U;
    Motor_SetPwm(motor_number, MOTOR_PWM_DUTY);
}

void Motor_Stop(uint8_t motor_number)
{
    Motor_DisableOutput(motor_number);
    motor.busy = 0U;
    motor.manual_mode = 0U;
    motor.manual_target = 0U;
}

MotorEvent Motor_RunOnce(void)
{
    uint8_t finished_motor;

    if (!motor.busy) {
        return MOTOR_EVENT_NONE;
    }

    Motor_WritePhase(motor.current_motor, motor.step_index);
    HAL_Delay(MOTOR_STEP_DELAY_MS);

    if (motor.manual_mode) {
        motor.step_index = (uint8_t)((motor.step_index + motor.manual_dir + 4) % 4);
        if (motor.manual_target > 0U) {
            motor.steps_done++;
            if (motor.steps_done >= motor.manual_target) {
                finished_motor = motor.manual_motor;
                Motor_Stop(finished_motor);
                return (finished_motor == 1U) ? MOTOR_EVENT_MANUAL1_DONE
                                              : MOTOR_EVENT_MANUAL2_DONE;
            }
        }
    } else {
        motor.step_index++;
        if (motor.step_index >= 4U) {
            motor.step_index = 0U;
        }
        motor.steps_done++;

        if (motor.steps_done >= motor.target_steps) {
            Motor_DisableOutput(motor.current_motor);
            motor.busy = 0U;
            return MOTOR_EVENT_AUTO_DONE;
        }
    }

    return MOTOR_EVENT_NONE;
}

uint8_t Motor_IsBusy(void)
{
    return motor.busy;
}
