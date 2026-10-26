#ifndef MODBUS_SENSOR_H
#define MODBUS_SENSOR_H

#include "main.h"

void ModbusSensor_Init(UART_HandleTypeDef *uart);
uint16_t ModbusSensor_CRC16(uint8_t *data, uint16_t length);
int ModbusSensor_ReadRegisters(uint8_t slave_addr,
                               uint16_t register_addr,
                               uint16_t register_count,
                               uint16_t *values);

#endif /* MODBUS_SENSOR_H */
