#include "step42_stm32_hal.h"

extern UART_HandleTypeDef huart2;

static Step42_STM32HAL_Handle stepper;

void Step42_STM32HAL_DemoInit(void)
{
    Step42_STM32HAL_Init(&stepper, &huart2, 100U);
}

void Step42_STM32HAL_DemoRun(void)
{
    Step42_STM32HAL_SetStream(&stepper, 0U);
    Step42_STM32HAL_Enable(&stepper, 1U);
    Step42_STM32HAL_SetMode(&stepper, STEP42_MODE_CLOSED_POSITION);
    Step42_STM32HAL_SetAngle(&stepper, 90.0f);
}

void Step42_STM32HAL_DemoSpeed(void)
{
    Step42_STM32HAL_Enable(&stepper, 1U);
    Step42_STM32HAL_SetMode(&stepper, STEP42_MODE_OPEN_SPEED);
    Step42_STM32HAL_SetSpeed(&stepper, 1000, 3000U);
}

void Step42_STM32HAL_DemoStop(void)
{
    Step42_STM32HAL_Stop(&stepper);
}
