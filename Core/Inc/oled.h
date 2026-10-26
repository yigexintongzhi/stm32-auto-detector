#ifndef OLED_H
#define OLED_H

#include "main.h"

typedef void (*OLED_I2C_ReinitCallback)(void);

void OLED_DriverInit(I2C_HandleTypeDef *i2c, OLED_I2C_ReinitCallback reinit_callback);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t page, uint8_t column, char *text);
void OLED_UpdateDisplay(uint16_t value, uint8_t success, uint16_t second);

#endif /* OLED_H */
