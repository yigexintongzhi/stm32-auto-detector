#ifndef CONSOLE_H
#define CONSOLE_H

#include "main.h"

typedef struct {
    uint8_t (*is_flow_idle)(void);
    uint8_t (*is_motor_busy)(void);
    uint8_t (*start_detection)(void);
    void (*motor_start)(uint8_t motor, int8_t direction, uint32_t target_steps);
    void (*motor_stop)(uint8_t motor);
    void (*pump_set)(uint8_t enabled);
    void (*set_stream)(uint8_t enabled);
    void (*slide_move)(int32_t steps);
    void (*slide_return_to_start)(void);
    int32_t (*slide_get_position)(void);
} ConsoleActions;

void Console_Init(UART_HandleTypeDef *uart, const ConsoleActions *actions);
void Console_RunOnce(void);
void Console_Print(char *text);
void Console_SendData(uint16_t *values, uint16_t point_count);

#endif /* CONSOLE_H */
