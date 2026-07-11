#include "Step42BoardMode.h"
#include "main.h"

static Step42BoardMode_t board_mode = STEP42_BOARD_UART;

void Step42BoardMode_Init(void)
{
    uint8_t ms1 = (HAL_GPIO_ReadPin(MS1_GPIO_Port, MS1_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t ms2 = (HAL_GPIO_ReadPin(MS2_GPIO_Port, MS2_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    board_mode = (Step42BoardMode_t)((ms1 << 1U) | ms2);
}

Step42BoardMode_t Step42BoardMode_Get(void)
{
    return board_mode;
}
