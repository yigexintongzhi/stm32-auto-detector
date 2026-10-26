#include "app.h"
#include "app_config.h"
#include "console.h"
#include "modbus_sensor.h"
#include "motor.h"
#include "oled.h"
#include "slide.h"
#include <stdio.h>

typedef enum {
    STATE_IDLE = 0,
    STATE_MOTOR_RUN1,
    STATE_PUMP1,
    STATE_DELAY1,
    STATE_MOTOR_RUN2,
    STATE_COUNTDOWN,
    STATE_PUMP2,
    STATE_RESULT
} SystemState;

typedef struct {
    SystemState state;
    uint8_t stage;
    uint8_t result_hold;
    uint8_t data_sent;
    uint16_t values_a[COUNTDOWN_POINTS_A];
    uint16_t values_b[COUNTDOWN_POINTS_B];
    uint16_t index_a;
    uint16_t index_b;
    uint32_t countdown_time_ms;
    uint16_t *current_values;
    uint16_t *current_index;
    uint32_t countdown_start;
    uint32_t phase_start;
    uint16_t adc_values[2];
    uint32_t last_read_time;
    uint8_t modbus_success;
    uint8_t key_last;
} AppContext;

static AppContext app;
static AppHardware app_hardware;

static uint8_t App_IsFlowIdle(void)
{
    return app.state == STATE_IDLE;
}

static void App_SetPump(uint8_t enabled)
{
    HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_RecoverDisplay(void)
{
    if (HAL_I2C_GetState(app_hardware.oled_i2c) != HAL_I2C_STATE_READY) {
        app_hardware.i2c1_reinit();
        OLED_Init();
        OLED_Clear();
    }
}

static void App_SelectStageA(void)
{
    app.stage = 1U;
    app.countdown_time_ms = COUNTDOWN_TIME_MS_A;
    app.current_values = app.values_a;
    app.current_index = &app.index_a;
    *app.current_index = 0U;
}

static void App_SelectStageB(void)
{
    app.stage = 2U;
    app.countdown_time_ms = COUNTDOWN_TIME_MS_B;
    app.current_values = app.values_b;
    app.current_index = &app.index_b;
    *app.current_index = 0U;
}

static void App_StartCurrentMotor(void)
{
    if (app.stage == 1U) {
        Motor_StartAutomatic(1U, MOTOR1_TARGET_STEPS);
    } else {
        Motor_StartAutomatic(2U, MOTOR2_TARGET_STEPS);
    }
}

static void App_HandleButton(void)
{
    uint8_t key_now = (uint8_t)HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);

    if (key_now == 1U && app.key_last == 0U) {
        HAL_Delay(KEY_DEBOUNCE_MS);
        if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_SET) {
            if (!Motor_IsBusy() && app.state == STATE_IDLE) {
                app.data_sent = 0U;
                app.result_hold = 0U;
                App_RecoverDisplay();

                if (Slide_GetPosition() != 0) {
                    OLED_Clear();
                    OLED_ShowString(0U, 0U, "Slide Home");
                    Slide_ReturnToStart();
                    OLED_Clear();
                }

                App_SelectStageA();
                app.state = STATE_MOTOR_RUN1;
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                App_SetPump(0U);
                App_StartCurrentMotor();
            }
        }
    }
    app.key_last = key_now;
}

static void App_HandleMotor(void)
{
    MotorEvent event = Motor_RunOnce();

    if (event == MOTOR_EVENT_MANUAL1_DONE) {
        Console_Print("OK: Motor1 Finish\r\n");
    } else if (event == MOTOR_EVENT_MANUAL2_DONE) {
        Console_Print("OK: Motor2 Finish\r\n");
    } else if (event == MOTOR_EVENT_AUTO_DONE) {
        if (app.state == STATE_MOTOR_RUN1) {
            app.state = STATE_PUMP1;
            app.phase_start = HAL_GetTick();
            App_SetPump(1U);
        } else if (app.state == STATE_MOTOR_RUN2) {
            app.state = STATE_COUNTDOWN;
            app.countdown_start = HAL_GetTick();
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }
    }
}

static void App_ShowStageAResult(void)
{
    uint16_t value = (app.index_a >= 120U) ? app.values_a[119]
                     : (app.index_a > 0U ? app.values_a[app.index_a - 1U] : 0U);
    float concentration = ((float)value - CAL_A_B) / CAL_A_K;
    char message[64];
    char number[16];

    OLED_Clear();
    OLED_ShowString(0U, 0U, "Range1");

    if (concentration < CAL_A_MIN_PPM) {
        OLED_ShowString(1U, 0U, "< LOD");
        sprintf(message, "Result A: x < 3 ppm (LOD), y=%u\r\n", value);
    } else if (concentration <= CAL_A_MAX_PPM) {
        sprintf(number, "%.2f", concentration);
        OLED_ShowString(1U, 0U, "x: ");
        OLED_ShowString(1U, 18U, number);
        OLED_ShowString(1U, 60U, "ppm");
        sprintf(message, "Result A: x = %.2f ppm, y=%u\r\n",
                concentration, value);
    } else {
        OLED_Clear();
        OLED_ShowString(0U, 0U, "Range2");
        sprintf(message, "Result A: x > 18 ppm (%.1f), goto dilution\r\n",
                concentration);
        Console_Print(message);
        OLED_ShowString(1U, 0U, "Slide <<");
        Slide_Move(-(int32_t)SLIDE_SHIFT_STEPS);

        App_SelectStageB();
        app.data_sent = 0U;
        app.state = STATE_MOTOR_RUN1;
        App_StartCurrentMotor();
        return;
    }

    Console_Print(message);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    app.result_hold = 1U;
    app.state = STATE_IDLE;
}

static void App_ShowStageBResult(void)
{
    uint16_t value = (app.index_b >= 20U) ? app.values_b[19]
                     : (app.index_b > 0U ? app.values_b[app.index_b - 1U] : 0U);
    float concentration = ((float)value - CAL_B_B) / CAL_B_K;
    char message[64];
    char number[16];

    OLED_Clear();
    OLED_ShowString(0U, 0U, "Range2");

    if (concentration > CAL_B_MAX_PPM) {
        OLED_ShowString(1U, 0U, "> RANGE");
        sprintf(message, "Result B: x > 65 ppm (over range), y=%u\r\n", value);
    } else if (concentration >= CAL_B_MIN_PPM) {
        sprintf(number, "%.2f", concentration);
        OLED_ShowString(1U, 0U, "x: ");
        OLED_ShowString(1U, 18U, number);
        OLED_ShowString(1U, 60U, "ppm");
        sprintf(message, "Result B: x = %.2f ppm, y=%u\r\n",
                concentration, value);
    } else {
        OLED_ShowString(1U, 0U, "Err");
        sprintf(message, "Result B: ERROR x < 15 (%.1f), y=%u\r\n",
                concentration, value);
    }

    Console_Print(message);
    if (Slide_GetPosition() != 0) {
        OLED_ShowString(2U, 0U, "Slide Home");
        Slide_ReturnToStart();
    }

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    app.result_hold = 1U;
    app.state = STATE_IDLE;
}

static void App_RunStateMachine(uint32_t now)
{
    switch (app.state) {
        case STATE_PUMP1:
            if (now - app.phase_start >= PUMP_RUN_TIME_MS) {
                App_SetPump(0U);
                app.state = STATE_DELAY1;
                app.phase_start = now;
            }
            break;

        case STATE_DELAY1:
            if (now - app.phase_start >= SETTLE_TIME_MS) {
                app.state = STATE_MOTOR_RUN2;
                App_StartCurrentMotor();
            }
            break;

        case STATE_MOTOR_RUN2:
            break;

        case STATE_PUMP2:
            if (now - app.phase_start >= PUMP_RUN_TIME_MS) {
                App_SetPump(0U);
                app.state = STATE_RESULT;
            }
            break;

        case STATE_RESULT:
            if (app.stage == 1U) {
                App_ShowStageAResult();
            } else {
                App_ShowStageBResult();
            }
            break;

        default:
            break;
    }
}

static void App_SampleSensor(uint32_t now)
{
    if (!Motor_IsBusy() &&
        (app.state == STATE_IDLE || app.state == STATE_COUNTDOWN) &&
        (now - app.last_read_time >= SENSOR_SAMPLE_TIME_MS)) {
        uint16_t max_points;

        app.last_read_time = now;
        if (ModbusSensor_ReadRegisters(0x01U, 0x0000U, 2U,
                                       app.adc_values) == 0) {
            app.modbus_success = 1U;
        } else {
            app.modbus_success = 0U;
        }

        /* 为兼容现有数据格式，通信失败时仍保存上一次读数。 */
        if (app.state == STATE_COUNTDOWN && app.current_index != NULL) {
            max_points = (uint16_t)(app.countdown_time_ms / 1000U);
            if (*app.current_index < max_points) {
                app.current_values[*app.current_index] = app.adc_values[0];
                (*app.current_index)++;
            }
        }

        if (app.state == STATE_IDLE && !app.result_hold) {
            OLED_UpdateDisplay(app.adc_values[0], app.modbus_success, 0U);
        } else if (app.state == STATE_COUNTDOWN) {
            uint16_t second = (uint16_t)((now - app.countdown_start) / 1000U + 1U);
            OLED_UpdateDisplay(app.adc_values[0], app.modbus_success, second);
        }
    }
}

static void App_CheckCountdown(uint32_t now)
{
    if (!Motor_IsBusy() && app.state == STATE_COUNTDOWN &&
        (now - app.countdown_start >= app.countdown_time_ms)) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        OLED_UpdateDisplay(app.adc_values[0], app.modbus_success,
                           (uint16_t)(app.countdown_time_ms / 1000U));

        if (!app.data_sent && app.current_values != NULL) {
            Console_SendData(app.current_values, *app.current_index);
            app.data_sent = 1U;
        }

        app.state = STATE_PUMP2;
        app.phase_start = now;
        App_SetPump(1U);
    }
}

void App_Init(const AppHardware *hardware)
{
    ConsoleActions console_actions;

    app_hardware = *hardware;

    Motor_Init(hardware->motor1_pwm, hardware->motor2_pwm);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    Slide_Init();
    app.key_last = 0U;
    ModbusSensor_Init(hardware->sensor_uart);

    OLED_DriverInit(hardware->oled_i2c, hardware->i2c1_reinit);
    OLED_Init();
    OLED_Clear();

    app.state = STATE_IDLE;
    app.stage = 1U;
    app.result_hold = 0U;
    app.data_sent = 0U;
    app.index_a = 0U;
    app.index_b = 0U;
    app.countdown_time_ms = 0U;
    app.current_values = NULL;
    app.current_index = NULL;
    app.countdown_start = 0U;
    app.phase_start = 0U;
    app.adc_values[0] = 0U;
    app.adc_values[1] = 0U;
    app.last_read_time = 0U;
    app.modbus_success = 0U;

    console_actions.is_flow_idle = App_IsFlowIdle;
    console_actions.is_motor_busy = Motor_IsBusy;
    console_actions.motor_start = Motor_StartManual;
    console_actions.motor_stop = Motor_Stop;
    console_actions.pump_set = App_SetPump;
    console_actions.slide_move = Slide_Move;
    console_actions.slide_return_to_start = Slide_ReturnToStart;
    console_actions.slide_get_position = Slide_GetPosition;
    Console_Init(hardware->console_uart, &console_actions);
}

void App_RunOnce(void)
{
    uint32_t now = HAL_GetTick();

    Console_RunOnce();
    App_HandleButton();
    App_HandleMotor();

    /* 电机和命令处理可能阻塞，按原程序位置刷新时间。 */
    now = HAL_GetTick();
    App_RunStateMachine(now);
    Slide_RunIdle(now);
    App_SampleSensor(now);

    /* OLED、Modbus 和滑台也可能阻塞，结束判断再次取最新时间。 */
    now = HAL_GetTick();
    App_CheckCountdown(now);
}
