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

## Software Architecture

固件采用单线程轮询和有限状态机。`main.c` 只负责 HAL、时钟、外设初始化和应用入口，业务功能按职责拆分：

```text
main.c
  └─ App_RunOnce()
       ├─ console.c        USART2 命令接收、解析和数据输出
       ├─ motor.c          两路 TB6612 四拍励磁与 PWM 控制
       ├─ slide.c          A4988 相对脉冲位置控制
       ├─ modbus_sensor.c  RS485 / Modbus RTU 数据采集
       └─ oled.c           SSD1306 显示
```

| Module | Responsibility |
|---|---|
| `app.c` | 检测流程状态机、双量程切换、采样缓存、结果计算和主循环调度 |
| `app_config.h` | GPIO、时序、阈值、标定系数和缓冲区配置 |
| `motor.c` | 两路步进电机四拍查表、PWM 启停、自动及手动目标拍数 |
| `slide.c` | A4988 脉冲输出、相对位置记账、返回初始位置和空闲释放 |
| `modbus_sensor.c` | Modbus RTU 查询帧、CRC16 和 RS485 半双工方向切换 |
| `oled.c` | SSD1306 初始化、字符显示和 I2C 失败后的重初始化重试 |
| `console.c` | UART 中断接收、命令分帧、解析和 PC 数据发送 |

每次主循环按固定顺序处理：

```text
串口命令 → 启动按键 → 电机节拍 → 流程状态机
         → 滑台空闲释放 → 传感器采样 → 倒计时完成检查
```

USART2 中断中只保存字节并更新时间戳，命令解析与硬件控制在主循环中执行：

```text
USART2_IRQHandler
  → HAL_UART_IRQHandler(&huart2)
  → HAL_UART_RxCpltCallback
  → console 命令缓冲区
```

## Features

### Process Scheduling

- 使用 8 状态有限状态机调度进样、泵液、延时、采集和结果判定流程
- 使用 `HAL_GetTick()` 完成秒级流程的非阻塞计时
- 主量程超限时自动移动滑台并进入高量程检测流程
- 通过指针切换采集数组和索引，使两套量程共用同一套采集逻辑
- 保留三次时间快照，避免电机、通信和显示阻塞影响流程结束判断

### Motor and Stage Control

- 使用定时器 PWM 和 GPIO 查表方式输出四拍励磁时序，驱动两路步进蠕动泵
- 通过 A4988 的 STEP/DIR/EN 接口控制丝杆滑台
- 实现目标励磁拍数运行、相对位置计数、返回初始位置及静止超时自动释放
- 停机时先将 PWM 归零，再清除四个相位引脚
- 滑台位置采用开环脉冲计数，不包含编码器、限位开关或原点传感器

当前电机代码中的 `step` 按励磁拍计数，不等同于换算后的机械整步。电机每拍仍有 1 ms 阻塞延时，滑台 STEP 高、低电平各保持 1 ms；秒级流程等待采用非阻塞状态机实现。

### RS485 and Modbus RTU

- 实现 Modbus RTU 主站查询帧组包及 CRC16 校验
- 实现 RS485 半双工收发方向切换
- 增加接收超时、CRC 错误及 UART 错误状态处理
- 保留通信失败时继续记录上一次采样值的现有数据格式

### UART Debug Interface

- UART 中断逐字节接收，中断回调只负责存储字节并更新时间戳
- 主循环采用“收到回车/换行或连续 50 ms 无新字节”判定完整命令
- 支持电机、水泵和滑台的独立调试命令

| Command | Function |
|---|---|
| `M1F` / `M1R` / `M1S` | 电机 1 正转 / 反转 / 停止 |
| `M2F` / `M2R` / `M2S` | 电机 2 正转 / 反转 / 停止 |
| `M1F500` / `M1R500` | 电机 1 按目标励磁拍数运行 |
| `M2F500` / `M2R500` | 电机 2 按目标励磁拍数运行 |
| `PON` / `POF` | 水泵开启 / 关闭 |
| `SR500` / `SL500` | 滑台向右 / 向左移动指定脉冲数 |
| `SH` | 按软件累计的相对位置返回初始位置 |
| `H` / `?` | 显示命令帮助 |

### OLED Display

- 编写 SSD1306 基础字符、数值和状态数据显示驱动
- I2C 发送失败后重新初始化外设和 OLED，并进行一次重试

## Current Process Parameters

| Parameter | Value |
|---|---:|
| 低量程电机目标 | 551 个励磁拍 |
| 高量程电机目标 | 2204 个励磁拍 |
| 两次泵液 | 各 5 s |
| 中间静置 | 1 s |
| 低量程采集 | 120 s / 120 点 |
| 高量程采集 | 20 s / 20 点 |
| 量程切换滑台位移 | 向左 105 个脉冲 |
| 滑台空闲释放 | 5 s |

## Hardware

- MCU: STM32F103C8T6, ARM Cortex-M3, 72 MHz
- Motor drivers: TB6612, A4988
- RS485 transceiver: MAX485
- Display: SSD1306 OLED over I2C
- Sensor chain: optical probe + analog-to-RS485 acquisition module

## Software Stack

C / STM32 HAL / STM32CubeMX / Modbus RTU / RS485 / Keil MDK-ARM / Git

模块化版本已使用 ARMCC 5.06 update 5 完成干净构建：

```text
Program Size: Code=21130 RO-data=1090 RW-data=60 ZI-data=2460
0 Error(s), 0 Warning(s)
```

## Repository Contents

- `Core/Inc`: 应用模块头文件及 STM32 HAL 配置
- `Core/Src`: 应用模块、入口、MSP 和中断处理代码
- `自适应.ioc`: STM32CubeMX 配置
- `.gitignore`: Keil、编辑器和操作系统生成文件排除规则

本仓库用于展示应用源码和 CubeMX 配置，不包含 STM32 HAL、CMSIS、Keil 工程文件及编译产物。完整工程使用 STM32CubeF1 V1.8.7 和 Keil ARMCC 5.06 update 5 构建。

## Implementation Notes

- 本项目仍在进行标定和系统联调，时序及标定参数可能继续调整
- 滑台“返回初始位置”依赖软件累计位置，不是基于传感器的机械寻零
- OLED 恢复策略是重新初始化并重试，不代表完整的 I2C 总线故障自愈机制
