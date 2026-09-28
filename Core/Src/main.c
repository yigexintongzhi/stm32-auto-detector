/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"   // 包含芯片头文件（引脚名、外设名都在里面定义）

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;      // I2C1 外设句柄（用来操作 OLED 屏幕）

TIM_HandleTypeDef htim1;      // 定时器1 句柄（产生电机②的 PWM 信号）
TIM_HandleTypeDef htim3;      // 定时器3 句柄（产生电机①的 PWM 信号）

UART_HandleTypeDef huart1;    // 串口1 句柄（接 RS485 传感器）
UART_HandleTypeDef huart2;    // 串口2 句柄（接电脑传数据）

/* USER CODE BEGIN PV */
// ===== 电机①流程（低量程）：采集 120 秒，共 120 个点 =====
#define COUNTDOWN_TIME_MS_A    120000                          // 采集总时长（毫秒）= 120 秒
#define COUNTDOWN_POINTS_A     (COUNTDOWN_TIME_MS_A / 1000)    // 每秒存一个点，算出共 120 个点

// ===== 电机②流程（超量程稀释后）：采集 40 秒，共 40 个点 =====
#define COUNTDOWN_TIME_MS_B    40000                           // 采集总时长（毫秒）= 40 秒
#define COUNTDOWN_POINTS_B     (COUNTDOWN_TIME_MS_B / 1000)    // 算出共 40 个点

// 两个流程各自的数据数组和索引
uint16_t countdownValues_A[COUNTDOWN_POINTS_A];  // 电机①的采集数据缓存（一排储物柜，每格存一个采样值）
uint16_t countdownValues_B[COUNTDOWN_POINTS_B];  // 电机②的采集数据缓存
uint16_t countdownIndex_A = 0;                   // 电机①缓存的"下一个空格子"编号
uint16_t countdownIndex_B = 0;                   // 电机②缓存的"下一个空格子"编号

// 记录当前采集使用的参数（启动流程时设置，让同一套采集代码适配两种流程）
uint32_t currentCountdownTimeMs = 0;     // 当前倒计时总时长（毫秒）
uint16_t *currentCountdownValues = NULL; // 指向当前使用的数据数组（指针=存地址的变量）
uint16_t *currentCountdownIndex = NULL;  // 指向当前使用的索引变量

uint8_t dataSent = 0;        // 数据发送标志：1 = 已发给电脑，防止重复发送
uint8_t current_motor = 1;   // 当前用哪台电机：1 = 电机①，2 = 电机②
uint8_t stage = 1;           // 当前是第几套流程：1 = 低量程流程，2 = 稀释流程
uint8_t resultHold = 0;      // 结果保持标志：1 = 冻结结果画面，不被刷新覆盖

// ===== 电机运行状态变量（做成全局：串口命令处理函数也要读写它们） =====
uint8_t  step_index = 0;       // 当前跳到四拍舞曲的第几拍（0-3 循环）
uint32_t steps_done = 0;       // 这次已经转了多少步
uint8_t  motor_busy = 0;       // 电机是否在转：1=在转，0=闲着
uint32_t target_steps = 0;     // 这次要转的目标步数（流程开始时设置）

// ===== 串口命令接收（串口2 中断收字节，主循环解析执行） =====
uint8_t  uart2_rx_byte = 0;      // 中断每次收到的 1 个字节
uint8_t  cmd_buf[16] = {0};      // 命令缓冲区：攒够一条命令再处理
uint8_t  cmd_len = 0;            // 缓冲区里已收到的字节数
volatile uint8_t cmd_ready = 0;  // 命令就绪标志：1 = 主循环可以去解析执行了

// ===== 手动控制电机状态（串口调试助手接管电机时用） =====
uint8_t manual_mode = 0;   // 手动模式：1 = 电机一直转，直到收到停止命令（不走自动流程的步数限制）
uint8_t manual_motor = 1;  // 手动控制哪台电机（1 = 电机①，2 = 电机②）
int8_t  manual_dir = 1;    // 手动转动方向：+1 = 正转（拍号递增），-1 = 反转（拍号递减）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);      // 声明：配置系统时钟（函数在后面）
static void MX_GPIO_Init(void);     // 声明：初始化引脚（函数在后面）
static void MX_TIM3_Init(void);     // 声明：初始化定时器3（函数在后面）
static void MX_USART1_UART_Init(void);  // 声明：初始化串口1（函数在后面）
static void MX_I2C1_Init(void);     // 声明：初始化I2C1（函数在后面）
static void MX_USART2_UART_Init(void);  // 声明：初始化串口2（函数在后面）
static void MX_TIM1_Init(void);     // 声明：初始化定时器1（函数在后面）
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdint.h>   // 固定宽度整数类型（uint8_t / uint16_t / uint32_t 等）
#include <stdio.h>    // 标准输入输出（sprintf 把数字转成文字用）
#include <string.h>   // 字符串处理（strlen 计算文字长度用）

// ========== 电机①（TB6612 驱动芯片）控制引脚 ==========
#define AIN1_PIN   GPIO_PIN_12   // 电机① A 相绕组控制脚 1（PB12）
#define AIN2_PIN   GPIO_PIN_13   // 电机① A 相绕组控制脚 2（PB13）
#define BIN1_PIN   GPIO_PIN_14   // 电机① B 相绕组控制脚 1（PB14）
#define BIN2_PIN   GPIO_PIN_15   // 电机① B 相绕组控制脚 2（PB15）
#define STBY_PIN   GPIO_PIN_5    // TB6612 待机/使能脚（PB5 上的电机芯片用，注意与滑台EN区分）

// ========== 电机②控制引脚 ==========
#define MOTOR2_AIN1_PORT   GPIOB           // 电机② AIN1 所在的端口
#define MOTOR2_AIN1_PIN    GPIO_PIN_0      // 电机② AIN1（PB0）
#define MOTOR2_AIN2_PORT   GPIOB           // 电机② AIN2 所在端口
#define MOTOR2_AIN2_PIN    GPIO_PIN_1      // 电机② AIN2（PB1）
#define MOTOR2_BIN1_PORT   GPIOA           // 电机② BIN1 所在端口
#define MOTOR2_BIN1_PIN    GPIO_PIN_1      // 电机② BIN1（PA1）
#define MOTOR2_BIN2_PORT   GPIOA           // 电机② BIN2 所在端口
#define MOTOR2_BIN2_PIN    GPIO_PIN_4      // 电机② BIN2（PA4）

// 按键 K2：按下启动整套检测流程
#define KEY2_PIN    GPIO_PIN_11   // K2 按键接在 PA11 引脚
#define KEY2_PORT   GPIOA         // K2 所在端口是 GPIOA

// LED 指示灯（PA0）：采集结束时点亮
#define LED_PIN     GPIO_PIN_0    // LED 接在 PA0 引脚
#define LED_PORT    GPIOA         // LED 所在端口是 GPIOA

// 水泵控制引脚（PB4）：置 1 抽水，置 0 停止
#define PUMP_PIN    GPIO_PIN_4    // 水泵接在 PB4 引脚
#define PUMP_PORT   GPIOB         // 水泵所在端口是 GPIOB

// 步进电机四拍时序表：每行是一个节拍，4 个数 = 4 个引脚（AIN1, AIN2, BIN1, BIN2）的高低电平
const uint8_t step_sequence[4][4] = {
    {1, 0, 1, 0},   // 第1拍：A相正向通电
    {1, 0, 0, 1},   // 第2拍：A正+B反通电（转子被吸过去一点）
    {0, 1, 0, 1},   // 第3拍：B相反向通电
    {0, 1, 1, 0}    // 第4拍：A反+B正通电（走完4拍=转一步）
};

// 步进电机运行参数
#define STEP_DELAY_MS  1       // 每拍之间延时 1 毫秒（决定电机转速）
#define PWM_DUTY       500     // PWM 占空比 500（范围0-1000，决定电机力气大小）

// 各流程的目标步数（电机转多少步后停止）
#define STEP_MOTOR1    551     // 电机①流程（低量程）：转 551 步
#define STEP_MOTOR2    2204    // 电机②流程（稀释后）：转 2204 步

// ========== 滑台（A4988 驱动器：STEP / DIR / EN 三根线控制） ==========
#define SLIDE_STEP_PORT   GPIOA              // 滑台 STEP（脉冲）脚所在端口
#define SLIDE_STEP_PIN    GPIO_PIN_7         // PA7 → A4988 STEP（每发一个脉冲走一小步）
#define SLIDE_DIR_PORT    GPIOB              // 滑台 DIR（方向）脚所在端口
#define SLIDE_DIR_PIN     GPIO_PIN_10        // PB10 → A4988 DIR（高电平右移，低电平左移）
#define SLIDE_EN_PORT     GPIOB              // 滑台 EN（使能）脚所在端口
#define SLIDE_EN_PIN      GPIO_PIN_11        // PB11 → A4988 EN（低电平=通电使能）
#define SLIDE_STEPS_SHIFT 95                 // 超标时滑台左移 95 步（标定出来约等于 2cm）
#define SLIDE_STEP_DELAY  1                  // STEP 脉冲间隔 1 毫秒（决定滑台移动速度）

uint32_t slide_position = 0;   // 滑台当前位置（相对初始位置走了多少步，0 = 初始位置）

// 静止自动断电：最后一次移动后超过 SLIDE_IDLE_OFF_MS 自动拉高 EN 断电，
// 靠丝杠自锁保持位置（省掉持续保持电流，电机不烫）；下次移动自动重新使能。
#define SLIDE_IDLE_OFF_MS  5000    // 静止 5000 毫秒（5 秒）后自动断电
uint32_t slide_last_move = 0;      // 最后一次移动时的系统时间（毫秒）
uint8_t  slide_en_on = 1;          // 滑台当前是否通电：1 = 通电，0 = 已断电

// EN 拉低 = 使能（线圈通电，有保持电流）
void Slide_Enable(void)
{
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_RESET);  // EN 脚输出低电平 = 通电
    slide_en_on = 1;                                                 // 记下"已通电"
}

// EN 拉高 = 断电（靠丝杠自锁保持位置）
void Slide_Release(void)
{
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_SET);    // EN 脚输出高电平 = 断电
    slide_en_on = 0;                                                 // 记下"已断电"
}

// 滑台移动：steps 为正向右、为负向左；移动后自动更新位置计数
void Slide_Move(int32_t steps)
{
    if (steps == 0) {        // 如果叫滑台走 0 步
        return;              // 什么都不做，直接退出函数
    }
    if (!slide_en_on) {      // 如果滑台之前被自动断电了（在睡觉）
        Slide_Enable();      // 先叫醒它（重新通电）
        HAL_Delay(10);       // 等 10 毫秒让驱动器稳定
    }
    HAL_GPIO_WritePin(SLIDE_DIR_PORT, SLIDE_DIR_PIN,
                      steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);    // 正数设高电平=右移，负数设低电平=左移
    uint32_t n = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps); // 取绝对值 = 要发多少个脉冲
    for (uint32_t i = 0; i < n; i++) {                               // 一个脉冲循环一次
        HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_SET);   // STEP 拉高（脉冲开始）
        HAL_Delay(SLIDE_STEP_DELAY);                                        // 保持 1 毫秒
        HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_RESET); // STEP 拉低（脉冲结束）
        HAL_Delay(SLIDE_STEP_DELAY);                                        // 歇 1 毫秒再发下一个
    }
    slide_position += (uint32_t)steps;   // 记账：累计位置（往左走会自动减回去）
    slide_last_move = HAL_GetTick();     // 记账：刷新"最后移动时间"，重新算 5 秒断电
}

// 滑台回到初始位置（已在初始位置则不动）
void Slide_GoHome(void)
{
    if (slide_position != 0) {                     // 如果不在起点
        Slide_Move(-(int32_t)slide_position);      // 朝反方向走相同的步数，正好回到 0
    }
}

// ========== RS485 通信相关定义 ==========
#define RS485_CTRL_PORT  GPIOB              // RS485 收发方向控制脚所在端口
#define RS485_CTRL_PIN   GPIO_PIN_5         // PB5：高电平=发送模式，低电平=接收模式

// Modbus 通信缓冲区
uint8_t txBuffer[8];    // 发送缓冲区（Modbus 请求帧固定 8 字节）
uint8_t rxBuffer[32];   // 接收缓冲区（够放传感器的回答）

// 系统状态机：用一个数字记住"流程走到哪一步了"
typedef enum {
    STATE_IDLE,       // 0：空闲，等按键
    STATE_MOTOR_RUN1, // 1：第一次电机运行（低量程=电机①，稀释流程=电机②）
    STATE_PUMP1,      // 2：水泵运行 10 秒
    STATE_DELAY1,     // 3：静等 1 秒
    STATE_MOTOR_RUN2, // 4：第二次电机运行（与第一次同一台电机）
    STATE_COUNTDOWN,  // 5：倒计时采集数据
    STATE_PUMP2,      // 6：水泵第二次运行 10 秒
    STATE_RESULT      // 7：结果计算与显示
} SystemState;

SystemState systemState = STATE_IDLE;          // 系统当前状态，刚上电是"空闲"
uint32_t countdownStartTime = 0;               // 倒计时开始那一刻的时间（毫秒）
uint32_t phaseStartTime = 0;                   // 水泵/延时阶段开始那一刻的时间（毫秒）

// 标定方程参数：仪器读数 y（ADC值）换算成浓度 x（ppm）
// 电机①流程：y = 15.59213 * x - 19.48466  →  x = (y + 19.48466) / 15.59213
#define CAL_A_K      15.59213f     // 流程A标定直线的斜率（出厂做实验拟合出来的）
#define CAL_A_B      (-19.48466f)  // 流程A标定直线的截距
// 电机②流程：y = 2.49739 * x + 96.09863   →  x = (y - 96.09863) / 2.49739
#define CAL_B_K      2.49739f      // 流程B标定直线的斜率
#define CAL_B_B      96.09863f     // 流程B标定直线的截距

// CRC16 校验计算（Modbus 协议规定的"数据指纹"算法）
uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0xFFFF;                          // CRC 初值，Modbus 规定必须是 0xFFFF
    for (uint16_t i = 0; i < len; i++) {            // 对数据里的每一个字节
        crc ^= pData[i];                            // 字节与 CRC 异或（按位不进位加法）
        for (uint8_t j = 0; j < 8; j++) {           // 对这个字节的每一位
            if (crc & 0x0001) {                     // 如果 CRC 最低位是 1
                crc = (crc >> 1) ^ 0xA001;          // 右移一位，再与 0xA001 异或（协议规定的多项式）
            } else {                                // 如果最低位是 0
                crc = crc >> 1;                     // 只右移一位
            }
        }
    }
    return crc;                                     // 返回算好的 16 位"指纹"
}

// 通过 Modbus 协议从传感器读保持寄存器
// 参数：从机地址、寄存器地址、读几个、结果存到哪；返回 0=成功，-1=接收失败，-2=校验失败
int ReadModbusRegisters(uint8_t slaveAddr, uint16_t regAddr, uint16_t numRegs, uint16_t *values)
{
    // 第 1 步：拼一个 Modbus 查询帧（共 8 字节）
    txBuffer[0] = slaveAddr;                    // 第1字节：叫哪台传感器（0x01）
    txBuffer[1] = 0x03;                         // 第2字节：功能码 0x03 = "我要读寄存器"
    txBuffer[2] = (regAddr >> 8) & 0xFF;        // 第3字节：寄存器地址的高 8 位
    txBuffer[3] = regAddr & 0xFF;               // 第4字节：寄存器地址的低 8 位
    txBuffer[4] = (numRegs >> 8) & 0xFF;        // 第5字节：要读几个的高 8 位
    txBuffer[5] = numRegs & 0xFF;               // 第6字节：要读几个的低 8 位
    uint16_t crc = Modbus_CRC16(txBuffer, 6);   // 对前 6 字节算"指纹"
    txBuffer[6] = crc & 0xFF;                   // 第7字节：指纹的低 8 位
    txBuffer[7] = (crc >> 8) & 0xFF;            // 第8字节：指纹的高 8 位

    // 第 2 步：RS485 一根线不能同时说和听，先切到"说"模式
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET);   // 控制脚拉高=发送模式
    HAL_Delay(1);                               // 等 1 毫秒让收发器稳定

    // 第 3 步：把查询帧发出去
    HAL_UART_Transmit(&huart1, txBuffer, 8, 100);                 // 串口1发 8 字节，最多等 100 毫秒
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);   // 死等发送彻底完成

    // 第 4 步：切回"听"模式，等传感器回答
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);  // 控制脚拉低=接收模式

    // 第 5 步：接收传感器的回答
    uint16_t expectedLen = 5 + 2 * numRegs;     // 回答长度 = 5 字节固定头 + 每个数据 2 字节
    if (HAL_UART_Receive(&huart1, rxBuffer, expectedLen, 200) != HAL_OK) {  // 收数据，最多等 200 毫秒
        // 接收超时或出错：清掉串口错误标志（防止下次通信卡死）
        __HAL_UART_CLEAR_OREFLAG(&huart1);      // 清"溢出"错误标志
        __HAL_UART_CLEAR_NEFLAG(&huart1);       // 清"噪声"错误标志
        __HAL_UART_CLEAR_FEFLAG(&huart1);       // 清"帧错误"标志
        return -1;                              // 返回 -1 = 没收到
    }

    // 第 6 步：对收到的数据重新算指纹，跟回答里带的指纹比对（防干扰）
    crc = Modbus_CRC16(rxBuffer, expectedLen - 2);                  // 对除最后2字节外的内容算指纹
    uint16_t recvCRC = rxBuffer[expectedLen - 2] | (rxBuffer[expectedLen - 1] << 8);  // 取回答里带的指纹
    if (crc != recvCRC) {                       // 两个指纹不一样
        return -2;                              // 返回 -2 = 数据被干扰了，不可用
    }

    // 第 7 步：校验通过，把回答里的数据拆出来存好
    for (uint16_t i = 0; i < numRegs; i++) {    // 每个寄存器 2 字节
        values[i] = (rxBuffer[3 + 2 * i] << 8) | rxBuffer[4 + 2 * i];   // 高字节拼低字节 = 一个 16 位数
    }
    return 0;   // 返回 0 = 一切顺利
}

// ========== OLED 显示驱动（SSD1306 芯片，I2C 接口） ==========
#define OLED_ADDR 0x78   // OLED 的 I2C 地址（8位写法，7位地址是 0x3C）

// OLED 相关函数提前声明（告诉编译器"这些函数后面会写"）
void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t data);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t col);
void OLED_ShowChar(uint8_t page, uint8_t col, char ch);
void OLED_ShowString(uint8_t page, uint8_t col, char *str);
void OLED_ShowNum(uint8_t page, uint8_t col, uint32_t num, uint8_t len);
void OLED_UpdateDisplay(uint16_t value, uint8_t success, uint16_t second);

// 向 OLED 发送一条命令（第一个字节 0x00 表示"后面是命令"）
void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};   // 打包：[0x00, 命令]
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100) != HAL_OK) {  // 通过I2C发给屏幕，失败则：
        HAL_I2C_DeInit(&hi2c1);     // 关掉 I2C
        MX_I2C1_Init();             // 重新初始化 I2C（救活卡死的总线）
        OLED_Init();                // 重新初始化屏幕
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100);  // 再发一次
    }
}

// 向 OLED 发送一个显示数据（第一个字节 0x40 表示"后面是数据"）
void OLED_WriteData(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};  // 打包：[0x40, 数据]
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100) != HAL_OK) {  // 发送，失败同样自救
        HAL_I2C_DeInit(&hi2c1);     // 关 I2C
        MX_I2C1_Init();             // 重开 I2C
        OLED_Init();                // 重开屏幕
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100);  // 再试一次
    }
}

// OLED 初始化：按 SSD1306 手册规定的顺序发一串固定配置命令
void OLED_Init(void)
{
    HAL_Delay(100);          // 等屏幕上电稳定（100 毫秒）
    OLED_WriteCmd(0xAE);     // 关显示（先黑屏，配置完再开）
    OLED_WriteCmd(0xD5);     // 接下来设"显示时钟分频"
    OLED_WriteCmd(0x80);     // 分频值 = 0x80（手册建议值）
    OLED_WriteCmd(0xA8);     // 接下来设"多路复用率"
    OLED_WriteCmd(0x3F);     // 0x3F = 63 → 64 行
    OLED_WriteCmd(0xD3);     // 接下来设"显示偏移"
    OLED_WriteCmd(0x00);     // 偏移 = 0（不偏移）
    OLED_WriteCmd(0x40);     // 起始行 = 第 0 行
    OLED_WriteCmd(0x8D);     // 接下来设"电荷泵"（屏幕内部升压电路）
    OLED_WriteCmd(0x14);     // 0x14 = 打开电荷泵
    OLED_WriteCmd(0x20);     // 接下来设"内存寻址模式"
    OLED_WriteCmd(0x00);     // 0x00 = 水平寻址
    OLED_WriteCmd(0xA1);     // 列重映射（左右翻转，配合屏幕安装方向）
    OLED_WriteCmd(0xC8);     // 行扫描方向反向（上下翻转）
    OLED_WriteCmd(0xDA);     // 接下来设 COM 引脚硬件配置
    OLED_WriteCmd(0x12);     // 0x12 = 手册建议值
    OLED_WriteCmd(0x81);     // 接下来设"对比度"（亮度）
    OLED_WriteCmd(0xCF);     // 0xCF = 对比度数值（越大越亮）
    OLED_WriteCmd(0xD9);     // 接下来设"预充电周期"
    OLED_WriteCmd(0xF1);     // 0xF1 = 手册建议值
    OLED_WriteCmd(0xDB);     // 接下来设 VCOMH 电平
    OLED_WriteCmd(0x40);     // 0x40 = 手册建议值
    OLED_WriteCmd(0xA4);     // 显示内容来自显存（关掉"全屏点亮"测试模式）
    OLED_WriteCmd(0xA6);     // 正常显示（1=亮点；0xA7 是反白显示）
    OLED_WriteCmd(0x2E);     // 关闭滚动
    OLED_WriteCmd(0xAF);     // 打开显示
    OLED_Clear();            // 清屏（把显存全清成空白）
}

// 清屏：把 8 页 × 128 列的显存全部写 0
void OLED_Clear(void)
{
    for (uint8_t page = 0; page < 8; page++) {      // 屏幕纵向分 8 页（每页 8 行像素，8×8=64 行）
        OLED_WriteCmd(0xB0 + page);                 // 选中第 page 页
        OLED_WriteCmd(0x00);                        // 列地址低 4 位 = 0
        OLED_WriteCmd(0x10);                        // 列地址高 4 位 = 0
        for (uint16_t i = 0; i < 128; i++) {        // 该页一共 128 列
            OLED_WriteData(0x00);                   // 写 0 = 这一列 8 个像素全灭
        }
    }
}

// 设置光标位置（page：0-7 页；col：0-127 列）
void OLED_SetCursor(uint8_t page, uint8_t col)
{
    OLED_WriteCmd(0xB0 + page);                     // 发"页地址"命令（0xB0 是第0页）
    OLED_WriteCmd((col & 0x0F) | 0x00);             // 发"列地址低 4 位"（& 0x0F 是取低4位）
    OLED_WriteCmd(((col >> 4) & 0x0F) | 0x10);      // 发"列地址高 4 位"（>> 4 是把高4位挪下来）
}

// 8x6 点阵 ASCII 字库：每个字符是一幅 8行×6列 的"点阵画"，用 6 个字节表示
// 按 ASCII 码顺序从空格(32)排到波浪号(126)，每行花括号里 6 个数就是这幅画
const uint8_t asc2_0806[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // 空格
    {0x00,0x00,0x00,0x2f,0x00,0x00}, // !
    {0x00,0x00,0x07,0x00,0x07,0x00}, // "
    {0x00,0x14,0x7f,0x14,0x7f,0x14}, // #
    {0x00,0x24,0x2a,0x7f,0x2a,0x12}, // $
    {0x00,0x62,0x64,0x08,0x13,0x23}, // %
    {0x00,0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x00,0x1c,0x22,0x41,0x00}, // (
    {0x00,0x00,0x41,0x22,0x1c,0x00}, // )
    {0x00,0x14,0x08,0x3E,0x08,0x14}, // *
    {0x00,0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x00,0x00,0xA0,0x60,0x00}, // ,
    {0x00,0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x00,0x60,0x60,0x00,0x00}, // .
    {0x00,0x20,0x10,0x08,0x04,0x02}, // /
    {0x00,0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x00,0x42,0x61,0x51,0x49,0x46}, // 2
    {0x00,0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x00,0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x00,0x27,0x45,0x45,0x45,0x39}, // 5
    {0x00,0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x00,0x01,0x71,0x09,0x05,0x03}, // 7
    {0x00,0x36,0x49,0x49,0x49,0x36}, // 8
    {0x00,0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x00,0x56,0x36,0x00,0x00}, // ;
    {0x00,0x08,0x14,0x22,0x41,0x00}, // <
    {0x00,0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x00,0x41,0x22,0x14,0x08}, // >
    {0x00,0x02,0x01,0x51,0x09,0x06}, // ?
    {0x00,0x32,0x49,0x59,0x51,0x3E}, // @
    {0x00,0x7C,0x12,0x11,0x12,0x7C}, // A
    {0x00,0x7F,0x49,0x49,0x49,0x36}, // B
    {0x00,0x3E,0x41,0x41,0x41,0x22}, // C
    {0x00,0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x00,0x7F,0x49,0x49,0x49,0x41}, // E
    {0x00,0x7F,0x09,0x09,0x09,0x01}, // F
    {0x00,0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x00,0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x00,0x41,0x7F,0x41,0x00}, // I
    {0x00,0x20,0x40,0x41,0x3F,0x01}, // J
    {0x00,0x7F,0x08,0x14,0x22,0x41}, // K
    {0x00,0x7F,0x40,0x40,0x40,0x40}, // L
    {0x00,0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x00,0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x00,0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x00,0x7F,0x09,0x09,0x09,0x06}, // P
    {0x00,0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x00,0x7F,0x09,0x19,0x29,0x46}, // R
    {0x00,0x46,0x49,0x49,0x49,0x31}, // S
    {0x00,0x01,0x01,0x7F,0x01,0x01}, // T
    {0x00,0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x00,0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x00,0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x00,0x63,0x14,0x08,0x14,0x63}, // X
    {0x00,0x07,0x08,0x70,0x08,0x07}, // Y
    {0x00,0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x00,0x7F,0x41,0x41,0x00}, // [
    {0x00,0x02,0x04,0x08,0x10,0x20}, // 反斜杠 \
    {0x00,0x00,0x41,0x41,0x7F,0x00}, // ]
    {0x00,0x04,0x02,0x01,0x02,0x04}, // ^
    {0x00,0x40,0x40,0x40,0x40,0x40}, // _
    {0x00,0x00,0x01,0x02,0x04,0x00}, // `
    {0x00,0x20,0x54,0x54,0x54,0x78}, // a
    {0x00,0x7F,0x48,0x44,0x44,0x38}, // b
    {0x00,0x38,0x44,0x44,0x44,0x20}, // c
    {0x00,0x38,0x44,0x44,0x48,0x7F}, // d
    {0x00,0x38,0x54,0x54,0x54,0x18}, // e
    {0x00,0x08,0x7E,0x09,0x01,0x02}, // f
    {0x00,0x18,0xA4,0xA4,0xA4,0x7C}, // g
    {0x00,0x7F,0x08,0x04,0x04,0x78}, // h
    {0x00,0x00,0x44,0x7D,0x40,0x00}, // i
    {0x00,0x40,0x80,0x84,0x7D,0x00}, // j
    {0x00,0x7F,0x10,0x28,0x44,0x00}, // k
    {0x00,0x00,0x41,0x7F,0x40,0x00}, // l
    {0x00,0x7C,0x04,0x18,0x04,0x78}, // m
    {0x00,0x7C,0x08,0x04,0x04,0x78}, // n
    {0x00,0x38,0x44,0x44,0x44,0x38}, // o
    {0x00,0xFC,0x24,0x24,0x24,0x18}, // p
    {0x00,0x18,0x24,0x24,0x24,0xFC}, // q
    {0x00,0x7C,0x08,0x04,0x04,0x08}, // r
    {0x00,0x48,0x54,0x54,0x54,0x20}, // s
    {0x00,0x04,0x3F,0x44,0x40,0x20}, // t
    {0x00,0x3C,0x40,0x40,0x20,0x7C}, // u
    {0x00,0x1C,0x20,0x40,0x20,0x1C}, // v
    {0x00,0x3C,0x40,0x30,0x40,0x3C}, // w
    {0x00,0x44,0x28,0x10,0x28,0x44}, // x
    {0x00,0x1C,0xA0,0xA0,0xA0,0x7C}, // y
    {0x00,0x44,0x64,0x54,0x4C,0x44}, // z
    {0x00,0x00,0x08,0x77,0x41,0x00}, // {
    {0x00,0x00,0x00,0x7F,0x00,0x00}, // |
    {0x00,0x00,0x41,0x77,0x08,0x00}, // }
    {0x00,0x02,0x01,0x02,0x01,0x00}, // ~
};

// 在指定页、列显示一个字符
void OLED_ShowChar(uint8_t page, uint8_t col, char ch)
{
    if (ch < 32 || ch > 126) {   // 如果字符不在"可打印范围"内
        ch = '?';                // 用问号代替，防止查表越界出错
    }
    ch -= 32;                    // ASCII 码减 32 = 在字库表中的行号（空格是第0行）
    OLED_SetCursor(page, col);   // 把光标挪到要画的位置
    for (uint8_t i = 0; i < 6; i++) {   // 这个字符的画是 6 列宽
        OLED_WriteData(asc2_0806[ch][i]);  // 把第 i 列的点阵数据发给屏幕
    }
}

// 在指定页、列开始显示一串字符
void OLED_ShowString(uint8_t page, uint8_t col, char *str)
{
    while (*str) {                 // *str 是"当前字符"，为 0（结尾符）时结束循环
        OLED_ShowChar(page, col, *str);  // 显示当前这个字符
        str++;                     // 指针向后挪一个，看下一个字符
        col += 6;                  // 光标右移 6 列（一个字符的宽度）
        if (col > 122) {           // 如果快超出屏幕右边缘了
            break;                 // 停止显示（防止跑到屏幕外错位）
        }
    }
}

// 显示无符号整数（转成十进制文字，不足 len 位前面补 0）
void OLED_ShowNum(uint8_t page, uint8_t col, uint32_t num, uint8_t len)
{
    char buf[12];                        // 临时小纸条，存转换出来的文字
    sprintf(buf, "%0*lu", len, num);     // 把数字格式化成文字，如 00325
    OLED_ShowString(page, col, buf);     // 当字符串显示出来
}

// 更新 OLED 主界面：显示 ADC 数值、通信状态、倒计时秒数
void OLED_UpdateDisplay(uint16_t value, uint8_t success, uint16_t second)
{
    OLED_ShowString(0, 0, "CH0:");       // 第0行写"CH0:"
    if (success) {                       // 通信成功
        OLED_ShowNum(0, 40, value, 5);   // 第0行第40列显示5位数字
        OLED_ShowString(1, 0, "Status: OK  ");   // 第1行写"Status: OK"
    } else {                             // 通信失败
        OLED_ShowString(0, 40, "-----");         // 数值位置画横线占位
        OLED_ShowString(1, 0, "Status: FAIL");   // 第1行写"Status: FAIL"
    }
    if (second > 0) {                    // 如果正在倒计时（秒数>0）
        OLED_ShowString(2, 0, "Time:");          // 第2行写"Time:"
        OLED_ShowNum(2, 40, second, 3);          // 显示3位秒数
        OLED_ShowString(2, 58, "s");             // 后面写个"s"
    } else {                             // 没在倒计时
        OLED_ShowString(2, 0, "        ");       // 第2行清空，防止残留旧数字
    }
}

// 通过串口 2（连电脑）把采集到的数据发出去
void SendDataToPC(uint16_t *values, uint16_t numPoints)
{
    char buffer[20];                                       // 临时小纸条
    char start[] = "\r\n--- Countdown Data ---\r\n";       // 开头标记文字
    HAL_UART_Transmit(&huart2, (uint8_t *)start, strlen(start), 100);   // 先发开头标记

    for (uint16_t i = 0; i < numPoints; i++) {             // 对每个数据
        sprintf(buffer, "%u", values[i]);                  // 数字转成文字
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);  // 发出去
        if (i < numPoints - 1) {                           // 如果不是最后一个
            HAL_UART_Transmit(&huart2, (uint8_t *)",", 1, 100);          // 后面加个逗号隔开
        } else {                                           // 如果是最后一个
            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);       // 换行结束
        }
    }
    char end[] = "--- End of Data ---\r\n";                // 结尾标记文字
    HAL_UART_Transmit(&huart2, (uint8_t *)end, strlen(end), 100);       // 发结尾标记
}

// ========== 串口手动控制电机 ==========
// 让指定电机停转：PWM 占空比归 0（撤劲），4 个控制脚全部拉低（放松线圈）
// 参数 motor：1 = 电机①，2 = 电机②
void Motor_Stop(uint8_t motor)
{
    if (motor == 1) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);   // 撤掉电机①的 PWM
        HAL_GPIO_WritePin(GPIOB, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN, GPIO_PIN_RESET);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   // 撤掉电机②的 PWM
        HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, GPIO_PIN_RESET);
    }
}

// 通过串口2 回发一行文字给电脑（串口调试助手上能看到）
void UART2_Print(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

// 串口命令帮助：按 H 回车时打印
void UART2_ShowHelp(void)
{
    UART2_Print("\r\n===== Motor Control Commands =====\r\n");
    UART2_Print(" M1F : Motor1 Forward\r\n");
    UART2_Print(" M1R : Motor1 Reverse\r\n");
    UART2_Print(" M1S : Motor1 Stop\r\n");
    UART2_Print(" M2F : Motor2 Forward\r\n");
    UART2_Print(" M2R : Motor2 Reverse\r\n");
    UART2_Print(" M2S : Motor2 Stop\r\n");
    UART2_Print(" H   : Show this help\r\n");
    UART2_Print("Tip: motor must be stopped & system IDLE\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();      // STM32 开机第一步：初始化硬件底座（必须最先调用）

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();   // 配置主频 72MHz（单片机的"心跳"速度）

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();           // 初始化各引脚（设成输出/输入模式）
  MX_TIM3_Init();           // 初始化定时器3（电机①的 PWM）
  MX_USART1_UART_Init();    // 初始化串口1（连传感器，9600波特率）
  MX_I2C1_Init();           // 初始化I2C1（连OLED屏幕）
  MX_USART2_UART_Init();    // 初始化串口2（连电脑，115200波特率）
  MX_TIM1_Init();           // 初始化定时器1（电机②的 PWM）
  /* USER CODE BEGIN 2 */
  // ---------- 上电初始化（只执行一次） ----------

  HAL_GPIO_WritePin(GPIOA, STBY_PIN, GPIO_PIN_SET);   // 拉高 TB6612 待机脚 = 唤醒电机驱动芯片
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);           // 启动定时器3的 PWM 输出（电机①）...
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);           // 启动定时器1的 PWM 输出（电机②）
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    // 但占空比设 0 → 电机②暂不转
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    // 占空比设 0 → 电机①暂不转（待命）

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);   // LED 灭（低电平=灭）

  // 滑台 A4988 三根脚给个初始状态
  HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_RESET);   // STEP 拉低（不发脉冲）
  HAL_GPIO_WritePin(SLIDE_DIR_PORT, SLIDE_DIR_PIN, GPIO_PIN_RESET);     // DIR 拉低（方向朝左）
  HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_RESET);       // EN 拉低 = 通电使能
  slide_position = 0;            // 位置计数清零（假定上电时滑台停在初始位置）
  slide_last_move = HAL_GetTick();   // 记录此刻时间，开始"静止5秒断电"计时

  // 电机运行状态变量（step_index / steps_done / motor_busy / target_steps）
  // 已改为全局变量（见文件开头），这里不再重复定义

  uint8_t key2_last = 0;         // 按键的"上一次状态"，用来对比发现"刚按下"的瞬间

  HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);   // RS485 切到"听"模式

  OLED_Init();     // 初始化 OLED 屏幕
  OLED_Clear();    // 清屏

  // Modbus 数据变量
  uint16_t adcValues[2] = {0};    // 存传感器回答的 2 个寄存器值
  uint32_t lastReadTime = 0;      // 上一次读传感器的时间
  uint8_t  modbusSuccess = 0;     // 上次读取是否成功（1=成功）

  // 系统状态机变量
  systemState = STATE_IDLE;       // 状态设为"空闲"
  countdownStartTime = 0;         // 倒计时尚未开始

  dataSent = 0;                   // 数据未发送

  // 开启串口2 接收中断：电脑串口调试助手发来的命令在这里被逐字节收进来
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);            // 设中断优先级（数字越小越优先）
  HAL_NVIC_EnableIRQ(USART2_IRQn);                    // 打开串口2 的中断开关
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);    // 竖起耳朵：开始监听第 1 个字节

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)    // 死循环：单片机一辈子在这里面转圈巡逻
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();   // 看现在几点了（开机至今的毫秒数），下面所有计时都用它

    // ========== 0. 串口命令处理（串口调试助手控制电机正/反/停） ==========
    if (cmd_ready) {                          // 中断里攒好了一条命令
      cmd_ready = 0;                          // 清标志（先清再处理，处理期间新命令不丢）
      cmd_buf[cmd_len] = 0;                   // 补个字符串结束符，方便比较

      if (cmd_len == 1 && (cmd_buf[0] == 'H' || cmd_buf[0] == 'h' || cmd_buf[0] == '?')) {
        UART2_ShowHelp();                     // H 或 ? ：打印命令帮助
      } else if (cmd_len >= 3 && (cmd_buf[0] == 'M' || cmd_buf[0] == 'm')) {
        uint8_t m = (cmd_buf[1] == '2') ? 2 : 1;   // 第2个字符选电机：'2'=电机②，其余按电机①
        char act = cmd_buf[2];                     // 第3个字符选动作
        if (act >= 'a' && act <= 'z') act -= 32;   // 小写转大写（m1f 和 M1F 都行）

        if (act == 'S') {                     // ---- 停止命令：任何时候都有效（安全优先） ----
          Motor_Stop(m);
          motor_busy = 0;                     // 电机标记为空闲
          manual_mode = 0;                    // 退出手动模式
          UART2_Print(m == 1 ? "OK: Motor1 Stop\r\n" : "OK: Motor2 Stop\r\n");
        } else if (act == 'F' || act == 'R') {   // ---- 正转 / 反转 ----
          // 只有在"系统空闲且电机没在转"时才允许手动开转，防止干扰自动检测流程
          if (systemState == STATE_IDLE && !motor_busy) {
            manual_mode = 1;                  // 进入手动模式：一直转，直到收到停止命令
            manual_motor = m;                 // 记住手动控制哪台电机
            manual_dir = (act == 'F') ? 1 : -1;   // F=正转（拍号+1），R=反转（拍号-1）
            current_motor = m;                // 引脚输出时按这台电机走
            motor_busy = 1;                   // 标记：电机开始转了
            steps_done = 0;                   // 步数清零（手动模式不倒计数，仅保持整洁）
            step_index = 0;                   // 节拍从头开始
            if (m == 1) {
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY);   // 给电机①上电 → 转！
              UART2_Print(act == 'F' ? "OK: Motor1 Forward\r\n" : "OK: Motor1 Reverse\r\n");
            } else {
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY);   // 给电机②上电 → 转！
              UART2_Print(act == 'F' ? "OK: Motor2 Forward\r\n" : "OK: Motor2 Reverse\r\n");
            }
          } else {
            UART2_Print("Busy: stop motor & wait flow IDLE first\r\n");
          }
        } else {                              // 动作字符不认识
          UART2_Print("Unknown command. Send 'H' for help.\r\n");
        }
      } else {                                // 既不是 H 也不是 M 开头的命令
        UART2_Print("Unknown command. Send 'H' for help.\r\n");
      }
      cmd_len = 0;                            // 命令处理完，清空缓冲区等下一条
    }

    // ========== 1. 按键检测（K2 = PA11） ==========
    uint8_t key2_now = HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);   // 读按键引脚现在的电平（1=按下，0=松开）

    // 现在是1且上次是0 = "刚按下"的这一瞬间（只触发一次，不会重复触发）
    if (key2_now == 1 && key2_last == 0) {
      HAL_Delay(20);                                             // 等 20 毫秒"消抖"（按键有机械抖动）
      if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == 1) {          // 再读一次，确认确实按着
        if (!motor_busy && systemState == STATE_IDLE) {          // 只有电机闲着且系统空闲时才允许启动
          dataSent = 0;                                          // 清"已发送"标志
          resultHold = 0;                                        // 解除结果冻结，允许屏幕刷新
          // 如果 I2C 总线之前卡死了，就重新初始化救活它
          if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) { // 检查 I2C 状态是否正常
            HAL_I2C_DeInit(&hi2c1);                              // 不正常：关掉
            MX_I2C1_Init();                                      // 重开
            OLED_Init();                                         // 屏幕也重新初始化
            OLED_Clear();                                        // 清屏
          }
          // 滑台回初始位置（保证每次从同一位置出发）
          if (slide_position != 0) {                             // 如果滑台不在起点
            OLED_Clear();                                        // 清屏准备显示提示
            OLED_ShowString(0, 0, "Slide Home");                 // 显示"Slide Home"（滑台回位中）
            Slide_GoHome();                                      // 真的移回去
            OLED_Clear();                                        // 到了，清掉提示
          }
          // ===== 设置低量程流程（流程A）的参数 =====
          stage = 1;                                             // 当前是流程①
          current_motor = 1;                                     // 用电机①
          target_steps = STEP_MOTOR1;                            // 目标 551 步
          currentCountdownTimeMs = COUNTDOWN_TIME_MS_A;          // 采集 120 秒
          currentCountdownValues = countdownValues_A;            // 数据存到 A 组储物柜
          currentCountdownIndex = &countdownIndex_A;             // 用 A 组的索引
          *currentCountdownIndex = 0;                            // 索引清零（从第0格开始存）
          systemState = STATE_MOTOR_RUN1;                        // 状态机出发：第一次电机运行
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // LED 灭
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);// 水泵关
          motor_busy = 1;                                        // 标记：电机开始转了
          steps_done = 0;                                        // 步数清零
          step_index = 0;                                        // 节拍从头开始
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY);// 给电机①上电（占空比500）→ 转！
        }
      }
    }
    key2_last = key2_now;   // 记住这次按键状态，下圈巡逻时当"上次"用

    // ========== 2. 电机转动（每巡逻一圈转一拍，转完为止） ==========
    if (motor_busy) {   // 如果电机正在转
      // 根据当前用哪台电机，把这一拍的 4 个 0/1 写到对应的 4 个引脚上
      if (current_motor == 1) {   // 电机①（4 个脚都在 GPIOB）
        HAL_GPIO_WritePin(GPIOB, AIN1_PIN, step_sequence[step_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, AIN2_PIN, step_sequence[step_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, BIN1_PIN, step_sequence[step_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, BIN2_PIN, step_sequence[step_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
      } else {                    // 电机②（4 个脚分散在 GPIOA/GPIOB）
        HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, step_sequence[step_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, step_sequence[step_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, step_sequence[step_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, step_sequence[step_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      // 上面每个三元表达式意思是：时序表里是1就输出高电平，是0就输出低电平

      HAL_Delay(STEP_DELAY_MS);   // 歇 1 毫秒（节拍速度，太快电机转不动）

      if (manual_mode) {
        // ===== 手动模式（串口调试助手控制）：不按目标步数停，一直转到收到停止命令 =====
        // 正转 = 拍号往后走，反转 = 拍号往回走（步进电机倒着走四拍 = 反向旋转）
        step_index = (uint8_t)((step_index + manual_dir + 4) % 4);
      } else {
        // ===== 自动流程模式：数步数，转够目标就停 =====
        step_index++;               // 拍号 +1（跳到下一拍）
        if (step_index >= 4) {      // 4 拍一轮
          step_index = 0;           // 回到第 0 拍继续循环
        }
        steps_done++;               // 步数 +1（每 4 拍实际转一步，这里按拍计数）

        if (steps_done >= target_steps) {   // 如果转够了目标步数
        // 停！先关 PWM（撤掉电机的劲）
        if (current_motor == 1) {
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    // 电机①占空比归 0
        } else {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    // 电机②占空比归 0
        }
        // 再把 4 个控制引脚全部拉低（彻底放松线圈）
        if (current_motor == 1) {
          HAL_GPIO_WritePin(GPIOB, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN, GPIO_PIN_RESET);
        } else {
          HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, GPIO_PIN_RESET);
        }
        motor_busy = 0;           // 标记：电机转完了

        // 根据现在是流程的哪一步，决定接下来干什么
        if (systemState == STATE_MOTOR_RUN1) {         // 如果是"第一次转完"
          systemState = STATE_PUMP1;                   // → 进入"水泵阶段"
          phaseStartTime = HAL_GetTick();              // 记下此刻，开始算 10 秒
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);   // 水泵通电，开始抽水
        } else if (systemState == STATE_MOTOR_RUN2) {  // 如果是"第二次转完"
          systemState = STATE_COUNTDOWN;               // → 进入"倒计时采集"
          countdownStartTime = HAL_GetTick();          // 记下此刻，开始算倒计时
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);   // LED 保持灭
        }
        }   // if (steps_done >= target_steps) 结束
      }     // else（自动流程模式）结束
    }       // if (motor_busy) 结束

    now = HAL_GetTick();   // 电机转起来很耗时，重新看时间（沧海桑田了）

    // ========== 3. 状态机处理（水泵、延时、结果计算） ==========
    switch (systemState) {   // 看现在是流程的哪一步
      case STATE_PUMP1:      // 水泵阶段：等 10 秒后关水泵
        if (now - phaseStartTime >= 10000) {             // 现在 − 开始 ≥ 10秒？
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);  // 到点了：关水泵
          systemState = STATE_DELAY1;                    // → 进入"静等 1 秒"
          phaseStartTime = now;                          // 重新记时
        }
        break;    // 跳出 switch

      case STATE_DELAY1:   // 静等 1 秒
        if (now - phaseStartTime >= 1000) {              // 满 1 秒了？
          systemState = STATE_MOTOR_RUN2;                // → 进入"第二次电机运行"
          motor_busy = 1;                                // 电机又要开始转了
          steps_done = 0;                                // 步数清零
          step_index = 0;                                // 拍号清零
          if (current_motor == 1) {                      // 流程A：还是电机①
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY);   // 上电，转！
          } else {                                       // 流程B：电机②
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY);   // 上电，转！
          }
        }
        break;

      case STATE_MOTOR_RUN2:
        // 电机转动由上面第 2 部分负责，这里什么都不用做
        break;

      case STATE_PUMP2:    // 水泵第二次抽水：等 10 秒
        if (now - phaseStartTime >= 10000) {             // 满 10 秒了？
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);  // 关水泵
          systemState = STATE_RESULT;                    // → 进入"结果计算"
        }
        break;

      case STATE_RESULT: { // 结果计算与判定（注意这里多了层大括号，为了在里面定义变量）
        char msg[64];    // 存要发给电脑的文字
        char fbuf[16];   // 存格式化好的数字文字
        if (stage == 1) {   // ===== 流程A（低量程）的结果 =====
          // 取最后一个采样点的数据（索引-1 就是最后一格；一个都没存就取 0）
          uint16_t y = (countdownIndex_A > 0) ? countdownValues_A[countdownIndex_A - 1] : 0;
          float x = ((float)y - CAL_A_B) / CAL_A_K;   // 代入方程①反解浓度 x = (y+19.48466)/15.59213
          OLED_Clear();                               // 清屏准备显示结果
          OLED_ShowString(0, 0, "Range1");            // 第0行写"Range1"（量程1）
          if (x < 1.0f) {                             // 浓度小于 1ppm
            OLED_ShowString(1, 0, "< LOD");           // 显示"< LOD"（低于检出限）
            sprintf(msg, "Result A: x < 1 ppm (LOD), y=%u\r\n", y);   // 组好发给电脑的文字
          } else if (x <= 21.0f) {                    // 浓度在 1~21ppm 之间 = 正常
            sprintf(fbuf, "%.2f", x);                 // 数字转成"12.34"这样的文字（保留2位小数）
            OLED_ShowString(1, 0, "x: ");             // 第1行写"x: "
            OLED_ShowString(1, 18, fbuf);             // 写上浓度数值
            OLED_ShowString(1, 60, "ppm");            // 写上单位"ppm"
            sprintf(msg, "Result A: x = %.2f ppm, y=%u\r\n", x, y);   // 组好发给电脑的文字
          } else {                                    // 浓度超过 21ppm = 超量程！
            OLED_Clear();                             // 清屏
            OLED_ShowString(0, 0, "Range2");          // 显示"Range2"（要换量程了）
            sprintf(msg, "Result A: x > 21 ppm (%.1f), goto dilution\r\n", x);  // 组文字
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);       // 发给电脑
            OLED_ShowString(1, 0, "Slide <<");        // 屏幕提示"滑台移动中"
            Slide_Move(-(int32_t)SLIDE_STEPS_SHIFT);  // 滑台左移 95 步（约2cm）到稀释工作位
            // ===== 切换到流程B（稀释流程）的参数 =====
            stage = 2;                                      // 现在是流程②
            current_motor = 2;                              // 换电机②
            target_steps = STEP_MOTOR2;                     // 目标 2204 步
            currentCountdownTimeMs = COUNTDOWN_TIME_MS_B;   // 采集 40 秒
            currentCountdownValues = countdownValues_B;     // 数据存 B 组储物柜
            currentCountdownIndex = &countdownIndex_B;      // 用 B 组索引
            *currentCountdownIndex = 0;                     // 索引清零
            dataSent = 0;                                   // 清"已发送"标志（新阶段要重新发数据）
            motor_busy = 1;                                 // 电机开始转
            steps_done = 0;                                 // 步数清零
            step_index = 0;                                 // 拍号清零
            systemState = STATE_MOTOR_RUN1;                 // 状态机重新出发：再走一遍 电机→水泵→延时→电机→倒计时
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY);   // 给电机②上电 → 转！
            break;    // 跳出 switch（稀释流程走起来了，下面流程A的收尾不执行）
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);   // 把流程A的结果发给电脑
        } else {   // ===== 流程B（稀释后）的结果 =====
          // 优先取第 20 秒（索引19）的采样；不够20个点就取最后一个；一个都没有就取0
          uint16_t y = (countdownIndex_B >= 20) ? countdownValues_B[19]
                      : (countdownIndex_B > 0 ? countdownValues_B[countdownIndex_B - 1] : 0);
          float x = ((float)y - CAL_B_B) / CAL_B_K;   // 代入方程②反解浓度 x = (y-96.09863)/2.49739
          OLED_Clear();                               // 清屏
          OLED_ShowString(0, 0, "Range2");            // 第0行写"Range2"
          if (x > 65.0f) {                            // 超过 65ppm = 超量程
            OLED_ShowString(1, 0, "> RANGE");         // 显示"> RANGE"
            sprintf(msg, "Result B: x > 65 ppm (over range), y=%u\r\n", y);   // 组文字
          } else if (x >= 15.0f) {                    // 15~65ppm = 正常
            sprintf(fbuf, "%.2f", x);                 // 数字转文字
            OLED_ShowString(1, 0, "x: ");             // "x: "
            OLED_ShowString(1, 18, fbuf);             // 数值
            OLED_ShowString(1, 60, "ppm");            // 单位
            sprintf(msg, "Result B: x = %.2f ppm, y=%u\r\n", x, y);   // 组文字
          } else {                                    // 低于 15ppm = 异常（稀释后不该这么低）
            OLED_ShowString(1, 0, "Err");             // 显示"Err"
            sprintf(msg, "Result B: ERROR x < 15 (%.1f), y=%u\r\n", x, y);    // 组文字
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);   // 把流程B的结果发给电脑
          // 流程全部结束：滑台回家
          if (slide_position != 0) {                  // 如果滑台不在起点
            OLED_ShowString(2, 0, "Slide Home");      // 第2行提示（不覆盖第1行的结果）
            Slide_GoHome();                           // 移回初始位置
          }
        }
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);   // 结果已出，LED 灭
        resultHold = 1;                                         // 冻结结果画面（别被后面的刷新覆盖）
        systemState = STATE_IDLE;                               // 回到"空闲"，等你下次按键
        break;
      }

      default:    // 其他状态（IDLE、COUNTDOWN）在这里没有要处理的事
        break;
    }

    // ========== 4. 滑台静止自动断电（省电降温） ==========
    if (slide_en_on && (now - slide_last_move >= SLIDE_IDLE_OFF_MS)) {  // 通电中且超5秒没动
      Slide_Release();   // 断电（丝杠自锁，位置不会跑）
    }

    // ========== 5. 每秒读一次传感器 ==========
    // 条件：电机闲着 + 系统处于"空闲"或"倒计时" + 距上次读取满 1 秒
    if (!motor_busy && (systemState == STATE_IDLE || systemState == STATE_COUNTDOWN)
        && (now - lastReadTime >= 1000)) {
      lastReadTime = now;   // 记下这次读取时间
      if (ReadModbusRegisters(0x01, 0x0000, 2, adcValues) == 0) {  // 问传感器（地址0x01）要 2 个寄存器
        modbusSuccess = 1;   // 读取成功，adcValues 里是新数据
      } else {
        modbusSuccess = 0;   // 读取失败，adcValues 里还是旧数据
      }

      // 倒计时期间：每秒把读数存进当前流程的储物柜（成功与否都占一格，失败存旧值）
      if (systemState == STATE_COUNTDOWN && currentCountdownIndex != NULL) {  // 在倒计时且指针有效
        uint16_t maxPoints = currentCountdownTimeMs / 1000;   // 当前流程最多存几个点（120或40）
        if (*currentCountdownIndex < maxPoints) {             // 储物柜还没满
          currentCountdownValues[*currentCountdownIndex] = adcValues[0];  // 把读数放进当前空格
          (*currentCountdownIndex)++;                                     // 空格编号+1
        }
      }

      // 更新 OLED 显示（结果冻结期间不刷新，防止覆盖结果画面）
      if (systemState == STATE_IDLE && !resultHold) {   // 空闲且没冻结
        OLED_UpdateDisplay(adcValues[0], modbusSuccess, 0);            // 刷新主界面（秒数传0）
      } else if (systemState == STATE_COUNTDOWN) {      // 倒计时中
        uint16_t currentSecond = (now - countdownStartTime) / 1000 + 1;   // 算出现在是第几秒
        OLED_UpdateDisplay(adcValues[0], modbusSuccess, currentSecond);   // 刷新并显示秒数
      }
    }

    // ========== 6. 倒计时结束检查 ==========
    now = HAL_GetTick();   // 再用最新时间检查一遍（前面可能干了很久的事）
    if (!motor_busy && systemState == STATE_COUNTDOWN) {   // 电机闲着且正在倒计时
      if ((now - countdownStartTime) >= currentCountdownTimeMs) {   // 倒计时够时间了？（120秒或40秒）
        // 1. 点亮 LED（告诉你"采完了"），屏幕定格在最后一秒
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        OLED_UpdateDisplay(adcValues[0], modbusSuccess, currentCountdownTimeMs / 1000);

        // 2. 把这一百来个采集数据发给电脑（只发一次）
        if (!dataSent && currentCountdownValues != NULL) {  // 还没发过且指针有效
          SendDataToPC(currentCountdownValues, *currentCountdownIndex);   // 发送！
          dataSent = 1;                                                 // 标记"已发"
        }

        // 3. 进入"水泵第二次抽水"阶段
        systemState = STATE_PUMP2;              // 换状态
        phaseStartTime = now;                   // 记时
        HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);   // 水泵通电
      }
    }

    // （结果计算状态不走这里：那时不读传感器，屏幕保持显示判定结果）

  }   // while(1) 循环结束——转回去，开始下一圈巡逻

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};     // 振荡器配置结构体（先清零）
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};     // 时钟分配结构体（先清零）

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;   // 用外部晶振（HSE）
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                     // 打开外部晶振
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;      // 晶振预分频 = 不分频
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     // 内部晶振也开着（备用）
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                 // 打开倍频器（PLL）
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;         // 倍频器输入接外部晶振
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;                 // 倍频 9 倍（8MHz×9=72MHz）
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)         // 应用振荡器配置
  {
    Error_Handler();    // 失败 = 晶振坏了，进入错误死循环
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK   // 要配置哪些时钟
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    // 主时钟来源 = 倍频器输出
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           // 总线不分频（满速 72MHz）
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;            // APB1 总线分一半（36MHz）
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            // APB2 总线不分频（72MHz）

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)   // 应用时钟配置
  {
    Error_Handler();    // 失败 = 进入错误死循环
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;                          // 用哪路 I2C 外设
  hi2c1.Init.ClockSpeed = 100000;                 // 通信速度 100kHz（标准模式）
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;         // 时钟占空比 2:1（100kHz 时用这个）
  hi2c1.Init.OwnAddress1 = 0;                     // 本机地址 0（单片机当主机，不用地址）
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;   // 7 位地址模式
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;  // 禁用双地址
  hi2c1.Init.OwnAddress2 = 0;                     // 第二地址 0（没启用）
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  // 禁用广播呼叫
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;      // 允许时钟拉伸（从机来不及可以拉慢时钟）
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)             // 应用配置，启动 I2C
  {
    Error_Handler();    // 失败 = 死循环报错
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};              // 主从配置结构体（清零）
  TIM_OC_InitTypeDef sConfigOC = {0};                       // 输出比较配置结构体（清零）
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};// 刹车/死区配置结构体（清零）

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;                            // 用定时器1
  htim1.Init.Prescaler = 71;                        // 预分频 71+1=72 → 每 1 微秒计一个数
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;      // 向上计数（0,1,2...往上数）
  htim1.Init.Period = 999;                          // 数到 999 归零（1000 微秒 = 1 毫秒一个周期）
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// 时钟不分频
  htim1.Init.RepetitionCounter = 0;                 // 重复计数 0（高级定时器专用，不用）
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // 不启用预装载
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)           // 按 PWM 模式启动定时器1
  {
    Error_Handler();    // 失败 = 死循环报错
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;        // 主触发输出 = 复位（不用触发别的外设）
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;  // 不用主从模式
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)   // 应用主从配置
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;               // 输出模式 = PWM 模式1
  sConfigOC.Pulse = 0;                              // 初始占空比 0（电机不转）
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;       // 有效电平 = 高
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;     // 互补通道有效电平 = 高
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;        // 不用快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;    // 空闲状态输出低
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;  // 互补空闲状态输出低
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)   // 应用到通道1
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;      // 运行态关断不用
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;     // 空闲态关断不用
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;           // 寄存器不锁定
  sBreakDeadTimeConfig.DeadTime = 0;                            // 死区时间 0（电机驱动不需要）
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;          // 刹车功能关闭
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;  // 刹车极性（没启用）
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;  // 自动输出关闭
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)   // 应用刹车配置
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);    // 把定时器引脚配置成 PWM 输出模式（在别的文件里）

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};    // 主从配置结构体（清零）
  TIM_OC_InitTypeDef sConfigOC = {0};             // 输出比较配置结构体（清零）

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;                            // 用定时器3
  htim3.Init.Prescaler = 71;                        // 预分频 72 → 每 1 微秒计一个数
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;      // 向上计数
  htim3.Init.Period = 999;                          // 数到 999 → 1 毫秒一个周期
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// 时钟不分频
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // 不启用预装载
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)           // 按 PWM 模式启动定时器3
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;        // 不触发别的外设
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;  // 不用主从模式
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)   // 应用配置
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;               // PWM 模式1
  sConfigOC.Pulse = 0;                              // 初始占空比 0（电机不转）
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;       // 有效电平 = 高
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;        // 不用快速模式
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)   // 应用到通道1
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);    // 把定时器引脚配置成 PWM 输出模式

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;                         // 用串口1
  huart1.Init.BaudRate = 9600;                      // 波特率 9600（和传感器约定好的速度）
  huart1.Init.WordLength = UART_WORDLENGTH_8B;      // 每个字节 8 位
  huart1.Init.StopBits = UART_STOPBITS_1;           // 1 个停止位
  huart1.Init.Parity = UART_PARITY_NONE;            // 无校验
  huart1.Init.Mode = UART_MODE_TX_RX;               // 既能发也能收
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // 不用硬件流控
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;  // 16 倍过采样（标准）
  if (HAL_UART_Init(&huart1) != HAL_OK)             // 应用配置，启动串口1
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;                         // 用串口2
  huart2.Init.BaudRate = 115200;                    // 波特率 115200（连电脑，快的）
  huart2.Init.WordLength = UART_WORDLENGTH_8B;      // 8 位
  huart2.Init.StopBits = UART_STOPBITS_1;           // 1 停止位
  huart2.Init.Parity = UART_PARITY_NONE;            // 无校验
  huart2.Init.Mode = UART_MODE_TX_RX;               // 收发都行
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // 不用流控
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;  // 16 倍过采样
  if (HAL_UART_Init(&huart2) != HAL_OK)             // 应用配置，启动串口2
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};   // 引脚配置结构体（先清零）
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();    // 打开 GPIO D 端口的时钟（不给时钟端口不工作）
  __HAL_RCC_GPIOA_CLK_ENABLE();    // 打开 GPIO A 端口的时钟
  __HAL_RCC_GPIOB_CLK_ENABLE();    // 打开 GPIO B 端口的时钟

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5   // 先把 GPIOA 的这些输出脚
                          |GPIO_PIN_7, GPIO_PIN_RESET);              // 全部拉低（初始安全状态）

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11   // 把 GPIOB 的这些输出脚
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);         // 全部拉低

  /*Configure GPIO pins : LED_Pin PA1 PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = LED_Pin|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5   // 要配置哪些引脚
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 模式 = 推挽输出（能拉高也能拉低）
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // 不上拉不下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // 翻转速度低（省电，灯/水泵不需要快）
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);       // 应用到 GPIOA

  /*Configure GPIO pins : PB0 PB1 PB10~PB15 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11   // 要配置哪些引脚
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // 无上拉下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // 低速
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);       // 应用到 GPIOB

  /*Configure GPIO pin : Key_Pin */
  GPIO_InitStruct.Pin = Key_Pin;                // 哪个引脚
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       // 模式 = 输入（读外面电平）
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;         // 下拉（平时读到 0，按下读到 1）
  HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);  // 应用

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // PA11 按键输入（K2：启动整套检测流程），带下拉
  // 注：即使 CubeMX 中未配置 PA11，这里也会强制初始化为下拉输入
  GPIO_InitStruct.Pin = GPIO_PIN_11;            // PA11
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       // 输入模式
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;         // 下拉（防悬空乱跳）
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);       // 应用

  // 滑台 A4988 三根信号线（PA7=STEP, PB10=DIR, PB11=EN）
  // 注：即使 CubeMX 中未配置这几个脚，这里也会强制初始化为输出
  GPIO_InitStruct.Pin = GPIO_PIN_7;             // PA7
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // 无上拉下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // 低速（滑台脉冲 1ms 足够）
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);       // 应用
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;  // PB10 和 PB11
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);       // 应用
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ========== 串口2 接收中断回调 ==========
// 每收到 1 个字节，HAL 自动调用一次这个函数（在中断里执行，所以要短小精悍，
// 只负责"存字节 + 竖标志"，真正的命令处理在主循环里做）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t b = uart2_rx_byte;                     // 取走刚收到的字节

        if (b == '\r' || b == '\n') {                  // 回车/换行 = 一条命令结束
            if (cmd_len > 0) {
                cmd_ready = 1;                         // 通知主循环：命令攒好了
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {    // 普通字符且缓冲区没满
            cmd_buf[cmd_len++] = b;                    // 存进缓冲区
            if (cmd_len >= 3) {                        // 凑够 3 个字母（如 M1F）立即执行
                cmd_ready = 1;                         // 串口助手不勾"发送新行"也能用
            }
        } else {                                       // 缓冲区塞满还没等到换行
            cmd_len = 0;                               // 丢弃，防止越界写坏内存
        }

        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);   // 再竖起耳朵听下一个字节
    }
}

// 串口出错（噪声/溢出/帧错误）时 HAL 会调用：清掉错误标志并重新收听，防止串口卡死
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();    // 关掉所有中断（停止一切活动，防止错误扩大）
  while (1)           // 死循环卡死在这里（相当于"亮红灯停摆"，等人工处理）
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
