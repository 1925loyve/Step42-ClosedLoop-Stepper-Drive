#ifndef STEP42_STM32_HAL_H
#define STEP42_STM32_HAL_H

#include "step42_protocol.h"

#ifndef STEP42_STM32_HAL_INCLUDE
#define STEP42_STM32_HAL_INCLUDE "main.h"
#endif
#include STEP42_STM32_HAL_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#define STEP42_HAL_OK          (0)
#define STEP42_HAL_ERROR      (-1)
#define STEP42_HAL_BAD_PARAM  (-2)
#define STEP42_HAL_BAD_FRAME  (-3)

typedef struct
{
    UART_HandleTypeDef *huart;
    uint32_t timeout_ms;
    uint8_t seq;
    Step42_Parser parser;
} Step42_STM32HAL_Handle;

void Step42_STM32HAL_Init(Step42_STM32HAL_Handle *handle,
                          UART_HandleTypeDef *huart,
                          uint32_t timeout_ms);

int Step42_STM32HAL_SendFrame(Step42_STM32HAL_Handle *handle,
                              uint8_t cmd,
                              const uint8_t *payload,
                              uint8_t payload_len);

int Step42_STM32HAL_Ping(Step42_STM32HAL_Handle *handle);
int Step42_STM32HAL_Enable(Step42_STM32HAL_Handle *handle, uint8_t enable);
int Step42_STM32HAL_SetMode(Step42_STM32HAL_Handle *handle, uint8_t mode);
int Step42_STM32HAL_SetAngle(Step42_STM32HAL_Handle *handle, float angle_deg);
int Step42_STM32HAL_SetSpeed(Step42_STM32HAL_Handle *handle, int16_t speed, uint16_t accel);
int Step42_STM32HAL_SetPID(Step42_STM32HAL_Handle *handle,
                           float kp,
                           float ki,
                           float kd,
                           uint16_t limit,
                           float deadband);
int Step42_STM32HAL_SetCurrent(Step42_STM32HAL_Handle *handle,
                               uint8_t hold_current,
                               uint8_t run_current);
int Step42_STM32HAL_SetStream(Step42_STM32HAL_Handle *handle, uint8_t enable);
int Step42_STM32HAL_Query(Step42_STM32HAL_Handle *handle);
int Step42_STM32HAL_Stop(Step42_STM32HAL_Handle *handle);

Step42_ParseResult Step42_STM32HAL_ParseByte(Step42_STM32HAL_Handle *handle,
                                             uint8_t byte,
                                             Step42_Response *response);

#ifdef __cplusplus
}
#endif

#endif
