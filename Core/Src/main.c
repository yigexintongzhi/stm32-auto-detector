/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define COUNTDOWN_TIME_MS_A    120000
#define COUNTDOWN_POINTS_A     (COUNTDOWN_TIME_MS_A / 1000)

#define COUNTDOWN_TIME_MS_B    40000
#define COUNTDOWN_POINTS_B     (COUNTDOWN_TIME_MS_B / 1000)

uint16_t countdownValues_A[COUNTDOWN_POINTS_A];
uint16_t countdownValues_B[COUNTDOWN_POINTS_B];
uint16_t countdownIndex_A = 0;
uint16_t countdownIndex_B = 0;

uint32_t currentCountdownTimeMs = 0;
uint16_t *currentCountdownValues = NULL;
uint16_t *currentCountdownIndex = NULL;

uint8_t dataSent = 0;
uint8_t current_motor = 1;
uint8_t stage = 1;
uint8_t resultHold = 0;

uint8_t  step_index = 0;
uint32_t steps_done = 0;
uint8_t  motor_busy = 0;
uint32_t target_steps = 0;

uint8_t  uart2_rx_byte = 0;
uint8_t  cmd_buf[16] = {0};
uint8_t  cmd_len = 0;
volatile uint8_t cmd_ready = 0;

uint8_t manual_mode = 0;
uint8_t manual_motor = 1;
int8_t  manual_dir = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AIN1_PIN   GPIO_PIN_12
#define AIN2_PIN   GPIO_PIN_13
#define BIN1_PIN   GPIO_PIN_14
#define BIN2_PIN   GPIO_PIN_15
#define STBY_PIN   GPIO_PIN_5

#define MOTOR2_AIN1_PORT   GPIOB
#define MOTOR2_AIN1_PIN    GPIO_PIN_0
#define MOTOR2_AIN2_PORT   GPIOB
#define MOTOR2_AIN2_PIN    GPIO_PIN_1
#define MOTOR2_BIN1_PORT   GPIOA
#define MOTOR2_BIN1_PIN    GPIO_PIN_1
#define MOTOR2_BIN2_PORT   GPIOA
#define MOTOR2_BIN2_PIN    GPIO_PIN_4

#define KEY2_PIN    GPIO_PIN_11
#define KEY2_PORT   GPIOA

#define LED_PIN     GPIO_PIN_0
#define LED_PORT    GPIOA

#define PUMP_PIN    GPIO_PIN_4
#define PUMP_PORT   GPIOB

const uint8_t step_sequence[4][4] = {
    {1, 0, 1, 0},
    {1, 0, 0, 1},
    {0, 1, 0, 1},
    {0, 1, 1, 0}
};

#define STEP_DELAY_MS  1
#define PWM_DUTY       500

#define STEP_MOTOR1    551
#define STEP_MOTOR2    2204

#define SLIDE_STEP_PORT   GPIOA
#define SLIDE_STEP_PIN    GPIO_PIN_7
#define SLIDE_DIR_PORT    GPIOB
#define SLIDE_DIR_PIN     GPIO_PIN_10
#define SLIDE_EN_PORT     GPIOB
#define SLIDE_EN_PIN      GPIO_PIN_11
#define SLIDE_STEPS_SHIFT 95
#define SLIDE_STEP_DELAY  1

uint32_t slide_position = 0;

#define SLIDE_IDLE_OFF_MS  5000
uint32_t slide_last_move = 0;
uint8_t  slide_en_on = 1;

void Slide_Enable(void)
{
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_RESET);
    slide_en_on = 1;
}

void Slide_Release(void)
{
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_SET);
    slide_en_on = 0;
}

void Slide_Move(int32_t steps)
{
    if (steps == 0) {
        return;
    }
    if (!slide_en_on) {
        Slide_Enable();
        HAL_Delay(10);
    }
    HAL_GPIO_WritePin(SLIDE_DIR_PORT, SLIDE_DIR_PIN,
                      steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uint32_t n = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps);
    for (uint32_t i = 0; i < n; i++) {
        HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(SLIDE_STEP_DELAY);
        HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(SLIDE_STEP_DELAY);
    }
    slide_position += (uint32_t)steps;
    slide_last_move = HAL_GetTick();
}

void Slide_GoHome(void)
{
    if (slide_position != 0) {
        Slide_Move(-(int32_t)slide_position);
    }
}

#define RS485_CTRL_PORT  GPIOB
#define RS485_CTRL_PIN   GPIO_PIN_5

uint8_t txBuffer[8];
uint8_t rxBuffer[32];

typedef enum {
    STATE_IDLE,
    STATE_MOTOR_RUN1,
    STATE_PUMP1,
    STATE_DELAY1,
    STATE_MOTOR_RUN2,
    STATE_COUNTDOWN,
    STATE_PUMP2,
    STATE_RESULT
} SystemState;

SystemState systemState = STATE_IDLE;
uint32_t countdownStartTime = 0;
uint32_t phaseStartTime = 0;

#define CAL_A_K      15.59213f
#define CAL_A_B      (-19.48466f)

#define CAL_B_K      2.49739f
#define CAL_B_B      96.09863f

uint16_t Modbus_CRC16(uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= pData[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

int ReadModbusRegisters(uint8_t slaveAddr, uint16_t regAddr, uint16_t numRegs, uint16_t *values)
{

    txBuffer[0] = slaveAddr;
    txBuffer[1] = 0x03;
    txBuffer[2] = (regAddr >> 8) & 0xFF;
    txBuffer[3] = regAddr & 0xFF;
    txBuffer[4] = (numRegs >> 8) & 0xFF;
    txBuffer[5] = numRegs & 0xFF;
    uint16_t crc = Modbus_CRC16(txBuffer, 6);
    txBuffer[6] = crc & 0xFF;
    txBuffer[7] = (crc >> 8) & 0xFF;

    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_UART_Transmit(&huart1, txBuffer, 8, 100);
    while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

    uint16_t expectedLen = 5 + 2 * numRegs;
    if (HAL_UART_Receive(&huart1, rxBuffer, expectedLen, 200) != HAL_OK) {

        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        return -1;
    }

    crc = Modbus_CRC16(rxBuffer, expectedLen - 2);
    uint16_t recvCRC = rxBuffer[expectedLen - 2] | (rxBuffer[expectedLen - 1] << 8);
    if (crc != recvCRC) {
        return -2;
    }

    for (uint16_t i = 0; i < numRegs; i++) {
        values[i] = (rxBuffer[3 + 2 * i] << 8) | rxBuffer[4 + 2 * i];
    }
    return 0;
}

#define OLED_ADDR 0x78

void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t data);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t col);
void OLED_ShowChar(uint8_t page, uint8_t col, char ch);
void OLED_ShowString(uint8_t page, uint8_t col, char *str);
void OLED_ShowNum(uint8_t page, uint8_t col, uint32_t num, uint8_t len);
void OLED_UpdateDisplay(uint16_t value, uint8_t success, uint16_t second);

void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100) != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
        OLED_Init();
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100);
    }
}

void OLED_WriteData(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};
    if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100) != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
        OLED_Init();
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buf, 2, 100);
    }
}

void OLED_Init(void)
{
    HAL_Delay(100);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0xD5);
    OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8);
    OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0xDA);
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81);
    OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9);
    OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0x2E);
    OLED_WriteCmd(0xAF);
    OLED_Clear();
}

void OLED_Clear(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        OLED_WriteCmd(0xB0 + page);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        for (uint16_t i = 0; i < 128; i++) {
            OLED_WriteData(0x00);
        }
    }
}

void OLED_SetCursor(uint8_t page, uint8_t col)
{
    OLED_WriteCmd(0xB0 + page);
    OLED_WriteCmd((col & 0x0F) | 0x00);
    OLED_WriteCmd(((col >> 4) & 0x0F) | 0x10);
}

const uint8_t asc2_0806[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x2f,0x00,0x00},
    {0x00,0x00,0x07,0x00,0x07,0x00},
    {0x00,0x14,0x7f,0x14,0x7f,0x14},
    {0x00,0x24,0x2a,0x7f,0x2a,0x12},
    {0x00,0x62,0x64,0x08,0x13,0x23},
    {0x00,0x36,0x49,0x55,0x22,0x50},
    {0x00,0x00,0x05,0x03,0x00,0x00},
    {0x00,0x00,0x1c,0x22,0x41,0x00},
    {0x00,0x00,0x41,0x22,0x1c,0x00},
    {0x00,0x14,0x08,0x3E,0x08,0x14},
    {0x00,0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x00,0x00,0xA0,0x60,0x00},
    {0x00,0x08,0x08,0x08,0x08,0x08},
    {0x00,0x00,0x60,0x60,0x00,0x00},
    {0x00,0x20,0x10,0x08,0x04,0x02},
    {0x00,0x3E,0x51,0x49,0x45,0x3E},
    {0x00,0x00,0x42,0x7F,0x40,0x00},
    {0x00,0x42,0x61,0x51,0x49,0x46},
    {0x00,0x21,0x41,0x45,0x4B,0x31},
    {0x00,0x18,0x14,0x12,0x7F,0x10},
    {0x00,0x27,0x45,0x45,0x45,0x39},
    {0x00,0x3C,0x4A,0x49,0x49,0x30},
    {0x00,0x01,0x71,0x09,0x05,0x03},
    {0x00,0x36,0x49,0x49,0x49,0x36},
    {0x00,0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x00,0x36,0x36,0x00,0x00},
    {0x00,0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41,0x00},
    {0x00,0x14,0x14,0x14,0x14,0x14},
    {0x00,0x00,0x41,0x22,0x14,0x08},
    {0x00,0x02,0x01,0x51,0x09,0x06},
    {0x00,0x32,0x49,0x59,0x51,0x3E},
    {0x00,0x7C,0x12,0x11,0x12,0x7C},
    {0x00,0x7F,0x49,0x49,0x49,0x36},
    {0x00,0x3E,0x41,0x41,0x41,0x22},
    {0x00,0x7F,0x41,0x41,0x22,0x1C},
    {0x00,0x7F,0x49,0x49,0x49,0x41},
    {0x00,0x7F,0x09,0x09,0x09,0x01},
    {0x00,0x3E,0x41,0x49,0x49,0x7A},
    {0x00,0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x00,0x41,0x7F,0x41,0x00},
    {0x00,0x20,0x40,0x41,0x3F,0x01},
    {0x00,0x7F,0x08,0x14,0x22,0x41},
    {0x00,0x7F,0x40,0x40,0x40,0x40},
    {0x00,0x7F,0x02,0x0C,0x02,0x7F},
    {0x00,0x7F,0x04,0x08,0x10,0x7F},
    {0x00,0x3E,0x41,0x41,0x41,0x3E},
    {0x00,0x7F,0x09,0x09,0x09,0x06},
    {0x00,0x3E,0x41,0x51,0x21,0x5E},
    {0x00,0x7F,0x09,0x19,0x29,0x46},
    {0x00,0x46,0x49,0x49,0x49,0x31},
    {0x00,0x01,0x01,0x7F,0x01,0x01},
    {0x00,0x3F,0x40,0x40,0x40,0x3F},
    {0x00,0x1F,0x20,0x40,0x20,0x1F},
    {0x00,0x3F,0x40,0x38,0x40,0x3F},
    {0x00,0x63,0x14,0x08,0x14,0x63},
    {0x00,0x07,0x08,0x70,0x08,0x07},
    {0x00,0x61,0x51,0x49,0x45,0x43},
    {0x00,0x00,0x7F,0x41,0x41,0x00},
    {0x00,0x02,0x04,0x08,0x10,0x20},
    {0x00,0x00,0x41,0x41,0x7F,0x00},
    {0x00,0x04,0x02,0x01,0x02,0x04},
    {0x00,0x40,0x40,0x40,0x40,0x40},
    {0x00,0x00,0x01,0x02,0x04,0x00},
    {0x00,0x20,0x54,0x54,0x54,0x78},
    {0x00,0x7F,0x48,0x44,0x44,0x38},
    {0x00,0x38,0x44,0x44,0x44,0x20},
    {0x00,0x38,0x44,0x44,0x48,0x7F},
    {0x00,0x38,0x54,0x54,0x54,0x18},
    {0x00,0x08,0x7E,0x09,0x01,0x02},
    {0x00,0x18,0xA4,0xA4,0xA4,0x7C},
    {0x00,0x7F,0x08,0x04,0x04,0x78},
    {0x00,0x00,0x44,0x7D,0x40,0x00},
    {0x00,0x40,0x80,0x84,0x7D,0x00},
    {0x00,0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x00,0x41,0x7F,0x40,0x00},
    {0x00,0x7C,0x04,0x18,0x04,0x78},
    {0x00,0x7C,0x08,0x04,0x04,0x78},
    {0x00,0x38,0x44,0x44,0x44,0x38},
    {0x00,0xFC,0x24,0x24,0x24,0x18},
    {0x00,0x18,0x24,0x24,0x24,0xFC},
    {0x00,0x7C,0x08,0x04,0x04,0x08},
    {0x00,0x48,0x54,0x54,0x54,0x20},
    {0x00,0x04,0x3F,0x44,0x40,0x20},
    {0x00,0x3C,0x40,0x40,0x20,0x7C},
    {0x00,0x1C,0x20,0x40,0x20,0x1C},
    {0x00,0x3C,0x40,0x30,0x40,0x3C},
    {0x00,0x44,0x28,0x10,0x28,0x44},
    {0x00,0x1C,0xA0,0xA0,0xA0,0x7C},
    {0x00,0x44,0x64,0x54,0x4C,0x44},
    {0x00,0x00,0x08,0x77,0x41,0x00},
    {0x00,0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x00,0x41,0x77,0x08,0x00},
    {0x00,0x02,0x01,0x02,0x01,0x00},
};

void OLED_ShowChar(uint8_t page, uint8_t col, char ch)
{
    if (ch < 32 || ch > 126) {
        ch = '?';
    }
    ch -= 32;
    OLED_SetCursor(page, col);
    for (uint8_t i = 0; i < 6; i++) {
        OLED_WriteData(asc2_0806[ch][i]);
    }
}

void OLED_ShowString(uint8_t page, uint8_t col, char *str)
{
    while (*str) {
        OLED_ShowChar(page, col, *str);
        str++;
        col += 6;
        if (col > 122) {
            break;
        }
    }
}

void OLED_ShowNum(uint8_t page, uint8_t col, uint32_t num, uint8_t len)
{
    char buf[12];
    sprintf(buf, "%0*lu", len, num);
    OLED_ShowString(page, col, buf);
}

void OLED_UpdateDisplay(uint16_t value, uint8_t success, uint16_t second)
{
    OLED_ShowString(0, 0, "CH0:");
    if (success) {
        OLED_ShowNum(0, 40, value, 5);
        OLED_ShowString(1, 0, "Status: OK  ");
    } else {
        OLED_ShowString(0, 40, "-----");
        OLED_ShowString(1, 0, "Status: FAIL");
    }
    if (second > 0) {
        OLED_ShowString(2, 0, "Time:");
        OLED_ShowNum(2, 40, second, 3);
        OLED_ShowString(2, 58, "s");
    } else {
        OLED_ShowString(2, 0, "        ");
    }
}

void SendDataToPC(uint16_t *values, uint16_t numPoints)
{
    char buffer[20];
    char start[] = "\r\n--- Countdown Data ---\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)start, strlen(start), 100);

    for (uint16_t i = 0; i < numPoints; i++) {
        sprintf(buffer, "%u", values[i]);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
        if (i < numPoints - 1) {
            HAL_UART_Transmit(&huart2, (uint8_t *)",", 1, 100);
        } else {
            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
        }
    }
    char end[] = "--- End of Data ---\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)end, strlen(end), 100);
}

void Motor_Stop(uint8_t motor)
{
    if (motor == 1) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOB, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN, GPIO_PIN_RESET);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, GPIO_PIN_RESET);
    }
}

void UART2_Print(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, STBY_PIN, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLIDE_DIR_PORT, SLIDE_DIR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_RESET);
  slide_position = 0;
  slide_last_move = HAL_GetTick();

  uint8_t key2_last = 0;

  HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

  OLED_Init();
  OLED_Clear();

  uint16_t adcValues[2] = {0};
  uint32_t lastReadTime = 0;
  uint8_t  modbusSuccess = 0;

  systemState = STATE_IDLE;
  countdownStartTime = 0;

  dataSent = 0;

  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if (cmd_ready) {
      cmd_ready = 0;
      cmd_buf[cmd_len] = 0;

      if (cmd_len == 1 && (cmd_buf[0] == 'H' || cmd_buf[0] == 'h' || cmd_buf[0] == '?')) {
        UART2_ShowHelp();
      } else if (cmd_len >= 3 && (cmd_buf[0] == 'M' || cmd_buf[0] == 'm')) {
        uint8_t m = (cmd_buf[1] == '2') ? 2 : 1;
        char act = cmd_buf[2];
        if (act >= 'a' && act <= 'z') act -= 32;

        if (act == 'S') {
          Motor_Stop(m);
          motor_busy = 0;
          manual_mode = 0;
          UART2_Print(m == 1 ? "OK: Motor1 Stop\r\n" : "OK: Motor2 Stop\r\n");
        } else if (act == 'F' || act == 'R') {

          if (systemState == STATE_IDLE && !motor_busy) {
            manual_mode = 1;
            manual_motor = m;
            manual_dir = (act == 'F') ? 1 : -1;
            current_motor = m;
            motor_busy = 1;
            steps_done = 0;
            step_index = 0;
            if (m == 1) {
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY);
              UART2_Print(act == 'F' ? "OK: Motor1 Forward\r\n" : "OK: Motor1 Reverse\r\n");
            } else {
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY);
              UART2_Print(act == 'F' ? "OK: Motor2 Forward\r\n" : "OK: Motor2 Reverse\r\n");
            }
          } else {
            UART2_Print("Busy: stop motor & wait flow IDLE first\r\n");
          }
        } else {
          UART2_Print("Unknown command. Send 'H' for help.\r\n");
        }
      } else {
        UART2_Print("Unknown command. Send 'H' for help.\r\n");
      }
      cmd_len = 0;
    }

    uint8_t key2_now = HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);

    if (key2_now == 1 && key2_last == 0) {
      HAL_Delay(20);
      if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == 1) {
        if (!motor_busy && systemState == STATE_IDLE) {
          dataSent = 0;
          resultHold = 0;

          if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
            HAL_I2C_DeInit(&hi2c1);
            MX_I2C1_Init();
            OLED_Init();
            OLED_Clear();
          }

          if (slide_position != 0) {
            OLED_Clear();
            OLED_ShowString(0, 0, "Slide Home");
            Slide_GoHome();
            OLED_Clear();
          }

          stage = 1;
          current_motor = 1;
          target_steps = STEP_MOTOR1;
          currentCountdownTimeMs = COUNTDOWN_TIME_MS_A;
          currentCountdownValues = countdownValues_A;
          currentCountdownIndex = &countdownIndex_A;
          *currentCountdownIndex = 0;
          systemState = STATE_MOTOR_RUN1;
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
          motor_busy = 1;
          steps_done = 0;
          step_index = 0;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY);
        }
      }
    }
    key2_last = key2_now;

    if (motor_busy) {

      if (current_motor == 1) {
        HAL_GPIO_WritePin(GPIOB, AIN1_PIN, step_sequence[step_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, AIN2_PIN, step_sequence[step_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, BIN1_PIN, step_sequence[step_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, BIN2_PIN, step_sequence[step_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, step_sequence[step_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, step_sequence[step_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, step_sequence[step_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, step_sequence[step_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

      HAL_Delay(STEP_DELAY_MS);

      if (manual_mode) {

        step_index = (uint8_t)((step_index + manual_dir + 4) % 4);
      } else {

        step_index++;
        if (step_index >= 4) {
          step_index = 0;
        }
        steps_done++;

        if (steps_done >= target_steps) {

        if (current_motor == 1) {
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        } else {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        }

        if (current_motor == 1) {
          HAL_GPIO_WritePin(GPIOB, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN, GPIO_PIN_RESET);
        } else {
          HAL_GPIO_WritePin(MOTOR2_AIN1_PORT, MOTOR2_AIN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_AIN2_PORT, MOTOR2_AIN2_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_BIN1_PORT, MOTOR2_BIN1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(MOTOR2_BIN2_PORT, MOTOR2_BIN2_PIN, GPIO_PIN_RESET);
        }
        motor_busy = 0;

        if (systemState == STATE_MOTOR_RUN1) {
          systemState = STATE_PUMP1;
          phaseStartTime = HAL_GetTick();
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);
        } else if (systemState == STATE_MOTOR_RUN2) {
          systemState = STATE_COUNTDOWN;
          countdownStartTime = HAL_GetTick();
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }
        }
      }
    }

    now = HAL_GetTick();

    switch (systemState) {
      case STATE_PUMP1:
        if (now - phaseStartTime >= 10000) {
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
          systemState = STATE_DELAY1;
          phaseStartTime = now;
        }
        break;

      case STATE_DELAY1:
        if (now - phaseStartTime >= 1000) {
          systemState = STATE_MOTOR_RUN2;
          motor_busy = 1;
          steps_done = 0;
          step_index = 0;
          if (current_motor == 1) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY);
          } else {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY);
          }
        }
        break;

      case STATE_MOTOR_RUN2:

        break;

      case STATE_PUMP2:
        if (now - phaseStartTime >= 10000) {
          HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_RESET);
          systemState = STATE_RESULT;
        }
        break;

      case STATE_RESULT: {
        char msg[64];
        char fbuf[16];
        if (stage == 1) {

          uint16_t y = (countdownIndex_A > 0) ? countdownValues_A[countdownIndex_A - 1] : 0;
          float x = ((float)y - CAL_A_B) / CAL_A_K;
          OLED_Clear();
          OLED_ShowString(0, 0, "Range1");
          if (x < 1.0f) {
            OLED_ShowString(1, 0, "< LOD");
            sprintf(msg, "Result A: x < 1 ppm (LOD), y=%u\r\n", y);
          } else if (x <= 21.0f) {
            sprintf(fbuf, "%.2f", x);
            OLED_ShowString(1, 0, "x: ");
            OLED_ShowString(1, 18, fbuf);
            OLED_ShowString(1, 60, "ppm");
            sprintf(msg, "Result A: x = %.2f ppm, y=%u\r\n", x, y);
          } else {
            OLED_Clear();
            OLED_ShowString(0, 0, "Range2");
            sprintf(msg, "Result A: x > 21 ppm (%.1f), goto dilution\r\n", x);
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
            OLED_ShowString(1, 0, "Slide <<");
            Slide_Move(-(int32_t)SLIDE_STEPS_SHIFT);

            stage = 2;
            current_motor = 2;
            target_steps = STEP_MOTOR2;
            currentCountdownTimeMs = COUNTDOWN_TIME_MS_B;
            currentCountdownValues = countdownValues_B;
            currentCountdownIndex = &countdownIndex_B;
            *currentCountdownIndex = 0;
            dataSent = 0;
            motor_busy = 1;
            steps_done = 0;
            step_index = 0;
            systemState = STATE_MOTOR_RUN1;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_DUTY);
            break;
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
        } else {

          uint16_t y = (countdownIndex_B >= 20) ? countdownValues_B[19]
                      : (countdownIndex_B > 0 ? countdownValues_B[countdownIndex_B - 1] : 0);
          float x = ((float)y - CAL_B_B) / CAL_B_K;
          OLED_Clear();
          OLED_ShowString(0, 0, "Range2");
          if (x > 65.0f) {
            OLED_ShowString(1, 0, "> RANGE");
            sprintf(msg, "Result B: x > 65 ppm (over range), y=%u\r\n", y);
          } else if (x >= 15.0f) {
            sprintf(fbuf, "%.2f", x);
            OLED_ShowString(1, 0, "x: ");
            OLED_ShowString(1, 18, fbuf);
            OLED_ShowString(1, 60, "ppm");
            sprintf(msg, "Result B: x = %.2f ppm, y=%u\r\n", x, y);
          } else {
            OLED_ShowString(1, 0, "Err");
            sprintf(msg, "Result B: ERROR x < 15 (%.1f), y=%u\r\n", x, y);
          }
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);

          if (slide_position != 0) {
            OLED_ShowString(2, 0, "Slide Home");
            Slide_GoHome();
          }
        }
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        resultHold = 1;
        systemState = STATE_IDLE;
        break;
      }

      default:
        break;
    }

    if (slide_en_on && (now - slide_last_move >= SLIDE_IDLE_OFF_MS)) {
      Slide_Release();
    }

    if (!motor_busy && (systemState == STATE_IDLE || systemState == STATE_COUNTDOWN)
        && (now - lastReadTime >= 1000)) {
      lastReadTime = now;
      if (ReadModbusRegisters(0x01, 0x0000, 2, adcValues) == 0) {
        modbusSuccess = 1;
      } else {
        modbusSuccess = 0;
      }

      if (systemState == STATE_COUNTDOWN && currentCountdownIndex != NULL) {
        uint16_t maxPoints = currentCountdownTimeMs / 1000;
        if (*currentCountdownIndex < maxPoints) {
          currentCountdownValues[*currentCountdownIndex] = adcValues[0];
          (*currentCountdownIndex)++;
        }
      }

      if (systemState == STATE_IDLE && !resultHold) {
        OLED_UpdateDisplay(adcValues[0], modbusSuccess, 0);
      } else if (systemState == STATE_COUNTDOWN) {
        uint16_t currentSecond = (now - countdownStartTime) / 1000 + 1;
        OLED_UpdateDisplay(adcValues[0], modbusSuccess, currentSecond);
      }
    }

    now = HAL_GetTick();
    if (!motor_busy && systemState == STATE_COUNTDOWN) {
      if ((now - countdownStartTime) >= currentCountdownTimeMs) {

        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        OLED_UpdateDisplay(adcValues[0], modbusSuccess, currentCountdownTimeMs / 1000);

        if (!dataSent && currentCountdownValues != NULL) {
          SendDataToPC(currentCountdownValues, *currentCountdownIndex);
          dataSent = 1;
        }

        systemState = STATE_PUMP2;
        phaseStartTime = now;
        HAL_GPIO_WritePin(PUMP_PORT, PUMP_PIN, GPIO_PIN_SET);
      }
    }

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin PA1 PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = LED_Pin|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10~PB15 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Key_Pin */
  GPIO_InitStruct.Pin = Key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t b = uart2_rx_byte;

        if (b == '\r' || b == '\n') {
            if (cmd_len > 0) {
                cmd_ready = 1;
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = b;
            if (cmd_len >= 3) {
                cmd_ready = 1;
            }
        } else {
            cmd_len = 0;
        }

        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    }
}

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
  __disable_irq();
  while (1)
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
