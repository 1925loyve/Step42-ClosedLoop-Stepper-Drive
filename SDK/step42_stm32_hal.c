#include "step42_stm32_hal.h"

static int Step42_STM32HAL_SendBuiltFrame(Step42_STM32HAL_Handle *handle,
                                          uint8_t *frame,
                                          uint16_t len)
{
    if ((handle == 0) || (handle->huart == 0) || (frame == 0))
    {
        return STEP42_HAL_BAD_PARAM;
    }
    if (len == 0U)
    {
        return STEP42_HAL_BAD_FRAME;
    }

    if (HAL_UART_Transmit(handle->huart, frame, len, handle->timeout_ms) != HAL_OK)
    {
        return STEP42_HAL_ERROR;
    }

    return STEP42_HAL_OK;
}

void Step42_STM32HAL_Init(Step42_STM32HAL_Handle *handle,
                          UART_HandleTypeDef *huart,
                          uint32_t timeout_ms)
{
    if (handle == 0)
    {
        return;
    }

    handle->huart = huart;
    handle->timeout_ms = timeout_ms;
    handle->seq = 0U;
    Step42_ParserInit(&handle->parser);
}

int Step42_STM32HAL_SendFrame(Step42_STM32HAL_Handle *handle,
                              uint8_t cmd,
                              const uint8_t *payload,
                              uint8_t payload_len)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildFrame(tx, sizeof(tx), cmd, ++handle->seq, payload, payload_len);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_Ping(Step42_STM32HAL_Handle *handle)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildPing(tx, sizeof(tx), ++handle->seq);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_Enable(Step42_STM32HAL_Handle *handle, uint8_t enable)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildEnable(tx, sizeof(tx), ++handle->seq, enable);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_SetMode(Step42_STM32HAL_Handle *handle, uint8_t mode)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildSetMode(tx, sizeof(tx), ++handle->seq, mode);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_SetAngle(Step42_STM32HAL_Handle *handle, float angle_deg)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildSetAngle(tx, sizeof(tx), ++handle->seq, angle_deg);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_SetSpeed(Step42_STM32HAL_Handle *handle, int16_t speed, uint16_t accel)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildSetSpeed(tx, sizeof(tx), ++handle->seq, speed, accel);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_SetPID(Step42_STM32HAL_Handle *handle,
                           float kp,
                           float ki,
                           float kd,
                           uint16_t limit,
                           float deadband)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildSetPID(tx, sizeof(tx), ++handle->seq, kp, ki, kd, limit, deadband);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_SetStream(Step42_STM32HAL_Handle *handle, uint8_t enable)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildSetStream(tx, sizeof(tx), ++handle->seq, enable);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_Query(Step42_STM32HAL_Handle *handle)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildQuery(tx, sizeof(tx), ++handle->seq);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

int Step42_STM32HAL_Stop(Step42_STM32HAL_Handle *handle)
{
    uint8_t tx[STEP42_MAX_FRAME_SIZE];
    uint16_t len;

    if (handle == 0)
    {
        return STEP42_HAL_BAD_PARAM;
    }

    len = Step42_BuildStop(tx, sizeof(tx), ++handle->seq);
    return Step42_STM32HAL_SendBuiltFrame(handle, tx, len);
}

Step42_ParseResult Step42_STM32HAL_ParseByte(Step42_STM32HAL_Handle *handle,
                                             uint8_t byte,
                                             Step42_Response *response)
{
    if (handle == 0)
    {
        return STEP42_PARSE_NONE;
    }

    return Step42_ParseByte(&handle->parser, byte, response);
}
