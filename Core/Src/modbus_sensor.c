#include "modbus_sensor.h"
#include "app_config.h"

static UART_HandleTypeDef *sensor_uart;
static uint8_t tx_buffer[8];
static uint8_t rx_buffer[32];

void ModbusSensor_Init(UART_HandleTypeDef *uart)
{
    sensor_uart = uart;
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);
}

uint16_t ModbusSensor_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if (crc & 0x0001U) {
                crc = (crc >> 1) ^ 0xA001U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int ModbusSensor_ReadRegisters(uint8_t slave_addr,
                               uint16_t register_addr,
                               uint16_t register_count,
                               uint16_t *values)
{
    uint16_t crc;
    uint16_t received_crc;
    uint16_t expected_length;
    uint16_t i;

    tx_buffer[0] = slave_addr;
    tx_buffer[1] = 0x03U;
    tx_buffer[2] = (uint8_t)((register_addr >> 8) & 0xFFU);
    tx_buffer[3] = (uint8_t)(register_addr & 0xFFU);
    tx_buffer[4] = (uint8_t)((register_count >> 8) & 0xFFU);
    tx_buffer[5] = (uint8_t)(register_count & 0xFFU);
    crc = ModbusSensor_CRC16(tx_buffer, 6U);
    tx_buffer[6] = (uint8_t)(crc & 0xFFU);
    tx_buffer[7] = (uint8_t)((crc >> 8) & 0xFFU);

    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_UART_Transmit(sensor_uart, tx_buffer, 8U, 100U);
    while (HAL_UART_GetState(sensor_uart) != HAL_UART_STATE_READY) {
    }
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

    expected_length = (uint16_t)(5U + 2U * register_count);
    if (HAL_UART_Receive(sensor_uart, rx_buffer, expected_length, 200U) != HAL_OK) {
        __HAL_UART_CLEAR_OREFLAG(sensor_uart);
        __HAL_UART_CLEAR_NEFLAG(sensor_uart);
        __HAL_UART_CLEAR_FEFLAG(sensor_uart);
        return -1;
    }

    crc = ModbusSensor_CRC16(rx_buffer, (uint16_t)(expected_length - 2U));
    received_crc = (uint16_t)(rx_buffer[expected_length - 2U] |
                              (rx_buffer[expected_length - 1U] << 8));
    if (crc != received_crc) {
        return -2;
    }

    for (i = 0U; i < register_count; i++) {
        values[i] = (uint16_t)((rx_buffer[3U + 2U * i] << 8) |
                               rx_buffer[4U + 2U * i]);
    }
    return 0;
}
