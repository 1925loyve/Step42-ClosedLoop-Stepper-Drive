#ifndef STEP42_COMMS_H
#define STEP42_COMMS_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Step42Comms_Init(void);
void Step42Comms_ProcessRx(void);
void Step42Comms_SendQuery(uint8_t seq);
void Step42Comms_SendStreamQueryIfDue(void);
void Step42Comms_OnUartError(UART_HandleTypeDef *huart);
void Step42Comms_OnBoardModeChanged(void);

#ifdef __cplusplus
}
#endif

#endif
