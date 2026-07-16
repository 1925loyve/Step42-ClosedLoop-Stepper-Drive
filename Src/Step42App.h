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
void Step42App_UARTRxCompleteCallback(UART_HandleTypeDef *huart);
void Step42App_FDCANRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its);
void Step42App_TIMCaptureCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif
