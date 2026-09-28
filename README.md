# STM32-based Automatic Biochemical Detection Instrument

An automatic detection instrument control system based on STM32F103,
integrating stepper motor motion control, RS485 sensor data acquisition,
and dual-range intelligent switching.

## Features

- Dual stepper motor control (four-phase excitation sequence + TB6612 PWM drive)
- Precision linear stage control via A4988 driver (STEP/DIR/EN interface),
  with position tracking, auto-homing and 5-second idle auto power-off
- RS485 half-duplex communication with Modbus RTU protocol (master),
  software-implemented CRC16 checksum, timeout & error recovery
- UART interrupt-driven command-line interface for motor debugging
- SSD1306 OLED display driver over I2C, with bus-stall self-recovery
- Finite state machine scheduling for the full detection workflow
- Dual-range auto-switching: automatic dilution path when primary range overflows

## Hardware

- MCU: STM32F103 (ARM Cortex-M3, 72 MHz)
- Motor drivers: TB6612 (dual DC/stepper driver), A4988 (stepper driver)
- Display: SSD1306 OLED (I2C)
- Sensor interface: RS485 transceiver + Modbus RTU sensor

## Tech Stack

C / STM32 HAL / CubeMX / Modbus RTU / RS485 / Keil
