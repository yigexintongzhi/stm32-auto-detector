#include "console.h"
#include "app_config.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *console_uart;
static ConsoleActions console_actions;
static uint8_t uart_rx_byte;
static uint8_t command_buffer[CONSOLE_BUFFER_SIZE];
static uint8_t command_length;
static volatile uint8_t command_ready;
static volatile uint32_t command_last_time;

static void Console_ShowHelp(void)
{
    Console_Print("\r\n===== Motor Control Commands =====\r\n");
    Console_Print(" RUN : Start full detection (as key press)\r\n");
    Console_Print(" M1F : Motor1 Forward (until M1S)\r\n");
    Console_Print(" M1F500 : Forward 500 steps then auto stop\r\n");
    Console_Print(" M1R : Motor1 Reverse (same, R = reverse)\r\n");
    Console_Print(" M2F / M2R / M2S : Motor2 (same as Motor1)\r\n");
    Console_Print(" PON : Pump ON\r\n");
    Console_Print(" POF : Pump OFF\r\n");
    Console_Print(" LON / LOF : Stream samples during countdown\r\n");
    Console_Print(" SR500 : Slide Right 500 steps\r\n");
    Console_Print(" SL500 : Slide Left 500 steps\r\n");
    Console_Print(" SH    : Slide go Home\r\n");
    Console_Print(" H   : Show this help\r\n");
    Console_Print("Tip: motor must be stopped & system IDLE\r\n");
}

static uint32_t Console_ParseNumber(uint8_t start,
                                    uint32_t maximum,
                                    uint8_t *has_number)
{
    uint32_t value = 0U;
    uint8_t i;

    *has_number = 0U;
    for (i = start; i < command_length; i++) {
        if (command_buffer[i] < '0' || command_buffer[i] > '9') {
            break;
        }
        value = value * 10U + (uint32_t)(command_buffer[i] - '0');
        *has_number = 1U;
        if (value > maximum) {
            value = maximum;
            break;
        }
    }
    return value;
}

static void Console_HandleMotor(void)
{
    uint8_t motor_number = (command_buffer[1] == '2') ? 2U : 1U;
    char action = (char)command_buffer[2];

    if (action >= 'a' && action <= 'z') {
        action = (char)(action - 32);
    }

    if (action == 'S') {
        console_actions.motor_stop(motor_number);
        Console_Print(motor_number == 1U ? "OK: Motor1 Stop\r\n"
                                         : "OK: Motor2 Stop\r\n");
    } else if (action == 'F' || action == 'R') {
        uint8_t has_number;
        uint32_t target = Console_ParseNumber(3U, CONSOLE_MOTOR_MAX_STEPS,
                                              &has_number);

        if (console_actions.is_flow_idle() && !console_actions.is_motor_busy()) {
            target = (has_number && target > 0U) ? target : 0U;
            console_actions.motor_start(motor_number,
                                        action == 'F' ? 1 : -1,
                                        target);
            if (target > 0U) {
                char buffer[40];
                sprintf(buffer, "OK: Motor%c %s %lu steps\r\n",
                        motor_number == 1U ? '1' : '2',
                        action == 'F' ? "Forward" : "Reverse",
                        target);
                Console_Print(buffer);
            } else {
                Console_Print(action == 'F'
                    ? (motor_number == 1U ? "OK: Motor1 Forward\r\n"
                                           : "OK: Motor2 Forward\r\n")
                    : (motor_number == 1U ? "OK: Motor1 Reverse\r\n"
                                           : "OK: Motor2 Reverse\r\n"));
            }
        } else {
            Console_Print("Busy: stop motor & wait flow IDLE first\r\n");
        }
    } else {
        Console_Print("Unknown command. Send 'H' for help.\r\n");
    }
}

static void Console_HandlePump(void)
{
    char first = (char)command_buffer[1];
    char second = (char)command_buffer[2];

    if (first >= 'a' && first <= 'z') {
        first = (char)(first - 32);
    }
    if (second >= 'a' && second <= 'z') {
        second = (char)(second - 32);
    }

    if (first == 'O' && second == 'N') {
        if (console_actions.is_flow_idle()) {
            console_actions.pump_set(1U);
            Console_Print("OK: Pump ON\r\n");
        } else {
            Console_Print("Busy: flow running, pump control locked\r\n");
        }
    } else if (first == 'O' && second == 'F') {
        console_actions.pump_set(0U);
        Console_Print("OK: Pump OFF\r\n");
    } else {
        Console_Print("Unknown command. Send 'H' for help.\r\n");
    }
}

static void Console_HandleLog(void)
{
    char first = (char)command_buffer[1];
    char second = (char)command_buffer[2];

    if (first >= 'a' && first <= 'z') {
        first = (char)(first - 32);
    }
    if (second >= 'a' && second <= 'z') {
        second = (char)(second - 32);
    }

    /* LON/LOF：控制倒计时期间是否每秒输出 STREAM 采样行。 */
    if (first == 'O' && second == 'N') {
        console_actions.set_stream(1U);
        Console_Print("OK: Stream ON\r\n");
    } else if (first == 'O' && second == 'F') {
        console_actions.set_stream(0U);
        Console_Print("OK: Stream OFF\r\n");
    } else {
        Console_Print("Unknown command. Send 'H' for help.\r\n");
    }
}

static void Console_HandleRun(void)
{
    char first = (char)command_buffer[1];
    char second = (char)command_buffer[2];

    if (first >= 'a' && first <= 'z') {
        first = (char)(first - 32);
    }
    if (second >= 'a' && second <= 'z') {
        second = (char)(second - 32);
    }

    /* RUN：等效按下物理启动按键，启动一次完整检测流程。 */
    if (first == 'U' && second == 'N') {
        if (console_actions.start_detection()) {
            Console_Print("OK: Detection started\r\n");
        } else {
            Console_Print("Busy: system not IDLE\r\n");
        }
    } else {
        Console_Print("Unknown command. Send 'H' for help.\r\n");
    }
}

static void Console_HandleSlide(void)
{
    char action = (char)command_buffer[1];

    if (action >= 'a' && action <= 'z') {
        action = (char)(action - 32);
    }

    if (action == 'H') {
        if (console_actions.is_flow_idle() && !console_actions.is_motor_busy()) {
            Console_Print("OK: Slide Homing...\r\n");
            console_actions.slide_return_to_start();
            Console_Print("OK: Slide Home done\r\n");
        } else {
            Console_Print("Busy: stop motor & wait flow IDLE first\r\n");
        }
    } else if (action == 'R' || action == 'L') {
        uint8_t has_number;
        uint32_t step_count = Console_ParseNumber(2U,
                                                   CONSOLE_SLIDE_MAX_STEPS,
                                                   &has_number);

        if (!has_number || step_count == 0U) {
            Console_Print("ERR: need steps, e.g. SR500 / SL200\r\n");
        } else if (!console_actions.is_flow_idle() ||
                   console_actions.is_motor_busy()) {
            Console_Print("Busy: stop motor & wait flow IDLE first\r\n");
        } else {
            int32_t movement = (action == 'R') ? (int32_t)step_count
                                               : -(int32_t)step_count;

            if (movement > 0 &&
                (console_actions.slide_get_position() + movement) > 0) {
                Console_Print("ERR: would pass home position (right limit)\r\n");
            } else {
                char buffer[48];
                sprintf(buffer, "OK: Slide %s %lu steps...\r\n",
                        action == 'R' ? "Right" : "Left", step_count);
                Console_Print(buffer);
                console_actions.slide_move(movement);
                sprintf(buffer, "OK: Slide done, pos=%ld\r\n",
                        (long)console_actions.slide_get_position());
                Console_Print(buffer);
            }
        }
    } else {
        Console_Print("Unknown command. Send 'H' for help.\r\n");
    }
}

void Console_Init(UART_HandleTypeDef *uart, const ConsoleActions *actions)
{
    console_uart = uart;
    console_actions = *actions;
    uart_rx_byte = 0U;
    command_length = 0U;
    command_ready = 0U;
    command_last_time = 0U;

    HAL_NVIC_SetPriority(USART2_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(console_uart, &uart_rx_byte, 1U);
}

void Console_RunOnce(void)
{
    if (!command_ready && command_length > 0U &&
        (HAL_GetTick() - command_last_time >= CONSOLE_IDLE_TIME_MS)) {
        command_ready = 1U;
    }

    if (!command_ready) {
        return;
    }

    command_ready = 0U;
    command_buffer[command_length] = 0U;

    if (command_length == 1U &&
        (command_buffer[0] == 'H' || command_buffer[0] == 'h' ||
         command_buffer[0] == '?')) {
        Console_ShowHelp();
    } else if (command_length >= 3U &&
               (command_buffer[0] == 'M' || command_buffer[0] == 'm')) {
        Console_HandleMotor();
    } else if (command_length >= 3U &&
               (command_buffer[0] == 'P' || command_buffer[0] == 'p')) {
        Console_HandlePump();
    } else if (command_length >= 3U &&
               (command_buffer[0] == 'L' || command_buffer[0] == 'l')) {
        Console_HandleLog();
    } else if (command_length >= 2U &&
               (command_buffer[0] == 'S' || command_buffer[0] == 's')) {
        Console_HandleSlide();
    } else if (command_length >= 3U &&
               (command_buffer[0] == 'R' || command_buffer[0] == 'r')) {
        Console_HandleRun();
    } else {
        Console_Print("Unknown command. Send 'H' for help.\r\n");
    }

    command_length = 0U;
}

void Console_Print(char *text)
{
    HAL_UART_Transmit(console_uart, (uint8_t *)text, strlen(text), 100U);
}

void Console_SendData(uint16_t *values, uint16_t point_count)
{
    char buffer[20];
    char start[] = "\r\n--- Countdown Data ---\r\n";
    char end[] = "--- End of Data ---\r\n";
    uint16_t i;

    HAL_UART_Transmit(console_uart, (uint8_t *)start, strlen(start), 100U);
    for (i = 0U; i < point_count; i++) {
        sprintf(buffer, "%u", values[i]);
        HAL_UART_Transmit(console_uart, (uint8_t *)buffer, strlen(buffer), 100U);
        if (i < point_count - 1U) {
            HAL_UART_Transmit(console_uart, (uint8_t *)",", 1U, 100U);
        } else {
            HAL_UART_Transmit(console_uart, (uint8_t *)"\r\n", 2U, 100U);
        }
    }
    HAL_UART_Transmit(console_uart, (uint8_t *)end, strlen(end), 100U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2) {
        uint8_t byte = uart_rx_byte;

        if (byte == '\r' || byte == '\n') {
            if (command_length > 0U) {
                command_ready = 1U;
            }
        } else if (command_length < sizeof(command_buffer) - 1U) {
            command_buffer[command_length++] = byte;
            command_last_time = HAL_GetTick();
        } else {
            command_length = 0U;
        }

        HAL_UART_Receive_IT(console_uart, &uart_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(uart);
        HAL_UART_Receive_IT(console_uart, &uart_rx_byte, 1U);
    }
}
