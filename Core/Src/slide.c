#include "slide.h"
#include "app_config.h"

static int32_t slide_position;
static uint32_t slide_last_move;
static uint8_t slide_en_on;

void Slide_Init(void)
{
    HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLIDE_DIR_PORT, SLIDE_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_RESET);
    slide_position = 0;
    slide_last_move = HAL_GetTick();
    slide_en_on = 1U;
}

void Slide_Enable(void)
{
    /* A4988 的 EN 为低电平有效。 */
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_RESET);
    slide_en_on = 1U;
}

void Slide_Release(void)
{
    HAL_GPIO_WritePin(SLIDE_EN_PORT, SLIDE_EN_PIN, GPIO_PIN_SET);
    slide_en_on = 0U;
}

void Slide_Move(int32_t steps)
{
    uint32_t count;
    uint32_t i;

    if (steps == 0) {
        return;
    }

    if (!slide_en_on) {
        Slide_Enable();
        HAL_Delay(10U);
    }

    HAL_GPIO_WritePin(SLIDE_DIR_PORT, SLIDE_DIR_PIN,
                      steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    count = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps);

    /* 保留阻塞式脉冲：高、低电平各维持 1 ms。 */
    for (i = 0U; i < count; i++) {
        HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(SLIDE_STEP_DELAY_MS);
        HAL_GPIO_WritePin(SLIDE_STEP_PORT, SLIDE_STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(SLIDE_STEP_DELAY_MS);
    }

    slide_position += steps;
    slide_last_move = HAL_GetTick();
}

void Slide_ReturnToStart(void)
{
    if (slide_position != 0) {
        Slide_Move(-slide_position);
    }
}

void Slide_RunIdle(uint32_t now)
{
    if (slide_en_on && (now - slide_last_move >= SLIDE_IDLE_OFF_MS)) {
        Slide_Release();
    }
}

int32_t Slide_GetPosition(void)
{
    return slide_position;
}
