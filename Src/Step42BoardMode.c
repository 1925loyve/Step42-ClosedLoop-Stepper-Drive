#include "Step42BoardMode.h"
#include "main.h"

static Step42BoardMode_t board_mode = STEP42_BOARD_UART;
static Step42BoardMode_t candidate_mode = STEP42_BOARD_UART;
static uint32_t candidate_since_ms = 0U;

#define BOARD_MODE_DEBOUNCE_MS (100UL)

static Step42BoardMode_t ReadBoardMode(void)
{
    uint8_t ms1 = (HAL_GPIO_ReadPin(MS1_GPIO_Port, MS1_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t ms2 = (HAL_GPIO_ReadPin(MS2_GPIO_Port, MS2_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    return (Step42BoardMode_t)((ms1 << 1U) | ms2);
}

void Step42BoardMode_Init(void)
{
    board_mode = ReadBoardMode();
    candidate_mode = board_mode;
    candidate_since_ms = HAL_GetTick();
}

uint8_t Step42BoardMode_Process(void)
{
    Step42BoardMode_t raw_mode = ReadBoardMode();
    uint32_t now = HAL_GetTick();

    if (raw_mode != candidate_mode)
    {
        candidate_mode = raw_mode;
        candidate_since_ms = now;
        return 0U;
    }
    if ((candidate_mode != board_mode) &&
        ((uint32_t)(now - candidate_since_ms) >= BOARD_MODE_DEBOUNCE_MS))
    {
        board_mode = candidate_mode;
        return 1U;
    }
    return 0U;
}

Step42BoardMode_t Step42BoardMode_Get(void)
{
    return board_mode;
}
