# STM32F103 Automatic Gas Detection Instrument Controller

基于 STM32F103 的全自动气体检测仪器控制系统，集成两路步进蠕动泵、A4988 丝杆滑台、光电探头、模拟量转 RS485 采集模块及 SSD1306 OLED，实现进样、泵液、数据采集、双量程切换和结果判定。

## System Architecture

```text
光电探头 → 模拟量转 RS485 采集模块 → MAX485 ↔ STM32F103
                                                ├─ TB6612 → 步进蠕动泵 1
                                                ├─ TB6612 → 步进蠕动泵 2
                                                ├─ A4988 → 丝杆滑台
                                                ├─ MOSFET → 水泵
                                                ├─ I2C → SSD1306 OLED
                                                └─ UART → PC 串口调试助手
```

## Features

### Process Scheduling

- 使用 8 状态有限状态机调度进样、泵液、延时、采集和结果判定流程
- 使用系统节拍完成秒级流程的非阻塞计时
- 主量程超限时自动移动滑台并进入高量程检测流程
- 通过流程参数指针切换，使两套量程共用同一套采集逻辑

### Motor and Stage Control

- 使用定时器 PWM 和 GPIO 查表方式输出四拍励磁时序，驱动两路步进蠕动泵
- 通过 A4988 的 STEP/DIR/EN 接口控制丝杆滑台
- 实现目标节拍运行、相对位置计数、返回初始位置及静止超时自动释放
- 滑台位置采用开环脉冲计数，不包含编码器、限位开关或原点传感器

### RS485 and Modbus RTU

- 实现 Modbus RTU 主站查询帧组包及 CRC16 校验
- 实现 RS485 半双工收发方向切换
- 增加接收超时、CRC 错误及 UART 错误状态处理

### UART Debug Interface

- UART 中断逐字节接收，中断回调只负责存储字节并更新时间戳
- 主循环采用“收到回车/换行或连续 50 ms 无新字节”判定完整命令
- 支持电机、水泵和滑台的独立调试命令

| Command | Function |
|---|---|
| `M1F` / `M1R` / `M1S` | 电机 1 正转 / 反转 / 停止 |
| `M2F` / `M2R` / `M2S` | 电机 2 正转 / 反转 / 停止 |
| `M1F500` / `M1R500` | 电机 1 按目标励磁节拍运行 |
| `M2F500` / `M2R500` | 电机 2 按目标励磁节拍运行 |
| `PON` / `POF` | 水泵开启 / 关闭 |
| `SR500` / `SL500` | 滑台向右 / 向左移动指定脉冲数 |
| `SH` | 按软件累计的相对位置返回初始位置 |
| `H` | 显示命令帮助 |

### OLED Display

- 编写 SSD1306 基础字符和数据显示驱动
- I2C 发送失败后重新初始化外设并进行重试

## Hardware

- MCU: STM32F103, ARM Cortex-M3, 72 MHz
- Motor drivers: TB6612, A4988
- RS485 transceiver: MAX485
- Display: SSD1306 OLED over I2C
- Sensor chain: optical probe + analog-to-RS485 acquisition module

## Software Stack

C / STM32 HAL / STM32CubeMX / Modbus RTU / RS485 / Keil / Git

## Repository Contents

- `Core/Inc`: application headers and STM32 HAL configuration
- `Core/Src`: application source and interrupt handlers
- `自适应.ioc`: STM32CubeMX project configuration

> The instrument is still under calibration and integration testing. Timing values, thresholds and calibration parameters may continue to change. The repository contains the application source and CubeMX configuration; STM32 HAL and CMSIS dependencies are not included.
