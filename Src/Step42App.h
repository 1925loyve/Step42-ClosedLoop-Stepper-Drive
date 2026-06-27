#ifndef STEP42_APP_H
#define STEP42_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Step42App_Init(void);
void Step42App_Loop(void);
void Step42App_TIMPeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Step42App_UARTErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
