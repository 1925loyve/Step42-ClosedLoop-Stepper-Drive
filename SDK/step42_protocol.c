#include "step42_protocol.h"

static void Step42_WriteU16(uint8_t *buf, uint8_t index, uint16_t value)
{
    buf[index] = (uint8_t)(value & 0xFFU);
    buf[index + 1U] = (uint8_t)(value >> 8);
}

static void Step42_WriteI16(uint8_t *buf, uint8_t index, int16_t value)
{
    Step42_WriteU16(buf, index, (uint16_t)value);
}

static void Step42_WriteI32(uint8_t *buf, uint8_t index, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    buf[index] = (uint8_t)(raw & 0xFFU);
    buf[index + 1U] = (uint8_t)((raw >> 8) & 0xFFU);
    buf[index + 2U] = (uint8_t)((raw >> 16) & 0xFFU);
    buf[index + 3U] = (uint8_t)((raw >> 24) & 0xFFU);
}

static uint16_t Step42_ReadU16(const uint8_t *buf, uint8_t index)
{
    return (uint16_t)buf[index] | ((uint16_t)buf[index + 1U] << 8);
}

static int16_t Step42_ReadI16(const uint8_t *buf, uint8_t index)
{
    return (int16_t)Step42_ReadU16(buf, index);
}

static uint16_t Step42_FloatToU16Scaled(float value, float scale)
{
    float scaled = value * scale;

    if (scaled <= 0.0f)
    {
        return 0U;
    }
    if (scaled >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(scaled + 0.5f);
}

static int16_t Step42_FloatToI16Scaled(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 32767.0f)
    {
        return 32767;
    }
    if (scaled <= -32768.0f)
    {
        return -32768;
    }
    if (scaled >= 0.0f)
    {
        return (int16_t)(scaled + 0.5f);
    }
    return (int16_t)(scaled - 0.5f);
}

uint8_t Step42_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00U;
    uint8_t bit;

    if (data == 0)
    {
        return 0U;
    }

    while (len-- > 0U)
    {
        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void Step42_ParserInit(Step42_Parser *parser)
{
    if (parser == 0)
    {
        return;
    }

    parser->state = 0U;
    parser->len = 0U;
    parser->cmd = 0U;
    parser->seq = 0U;
    parser->index = 0U;
}

Step42_ParseResult Step42_ParseByte(Step42_Parser *parser,
                                    uint8_t byte,
                                    Step42_Response *response)
{
    uint8_t crc_buf[STEP42_MAX_PAYLOAD + 3U];
    uint8_t crc;
    uint8_t i;

    if ((parser == 0) || (response == 0))
    {
        return STEP42_PARSE_NONE;
    }

    switch (parser->state)
    {
    case 0:
        if (byte == STEP42_RESP_SOF1)
        {
            parser->state = 1U;
        }
        return STEP42_PARSE_NONE;

    case 1:
        if (byte == STEP42_RESP_SOF2)
        {
            parser->state = 2U;
        }
        else
        {
            parser->state = 0U;
        }
        return STEP42_PARSE_NONE;

    case 2:
        parser->len = byte;
        parser->index = 0U;
        if (parser->len > STEP42_MAX_PAYLOAD)
        {
            parser->state = 0U;
            return STEP42_PARSE_LENGTH_ERROR;
        }
        parser->state = 3U;
        return STEP42_PARSE_NONE;

    case 3:
        parser->cmd = byte;
        parser->state = 4U;
        return STEP42_PARSE_NONE;

    case 4:
        parser->seq = byte;
        parser->state = (parser->len == 0U) ? 6U : 5U;
        return STEP42_PARSE_NONE;

    case 5:
        parser->payload[parser->index++] = byte;
        if (parser->index >= parser->len)
        {
            parser->state = 6U;
        }
        return STEP42_PARSE_NONE;

    case 6:
        crc_buf[0] = parser->len;
        crc_buf[1] = parser->cmd;
        crc_buf[2] = parser->seq;
        for (i = 0U; i < parser->len; i++)
        {
            crc_buf[3U + i] = parser->payload[i];
        }

        crc = Step42_Crc8(crc_buf, (uint8_t)(parser->len + 3U));
        if (crc != byte)
        {
            parser->state = 0U;
            return STEP42_PARSE_CRC_ERROR;
        }

        response->cmd = (uint8_t)(parser->cmd & 0x7FU);
        response->seq = parser->seq;
        response->len = parser->len;
        for (i = 0U; i < parser->len; i++)
        {
            response->payload[i] = parser->payload[i];
        }

        parser->state = 0U;
        return STEP42_PARSE_FRAME;

    default:
        parser->state = 0U;
        return STEP42_PARSE_NONE;
    }
}

uint16_t Step42_BuildFrame(uint8_t *buf,
                           uint16_t max_len,
                           uint8_t cmd,
                           uint8_t seq,
                           const uint8_t *payload,
                           uint8_t payload_len)
{
    uint8_t crc_buf[STEP42_MAX_PAYLOAD + 3U];
    uint8_t i;

    if ((buf == 0) ||
        (payload_len > STEP42_MAX_PAYLOAD) ||
        (max_len < (uint16_t)(payload_len + 6U)) ||
        ((payload_len > 0U) && (payload == 0)))
    {
        return 0U;
    }

    buf[0] = STEP42_SOF1;
    buf[1] = STEP42_SOF2;
    buf[2] = payload_len;
    buf[3] = cmd;
    buf[4] = seq;

    crc_buf[0] = payload_len;
    crc_buf[1] = cmd;
    crc_buf[2] = seq;

    for (i = 0U; i < payload_len; i++)
    {
        buf[5U + i] = payload[i];
        crc_buf[3U + i] = payload[i];
    }

    buf[5U + payload_len] = Step42_Crc8(crc_buf, (uint8_t)(payload_len + 3U));
    return (uint16_t)(payload_len + 6U);
}

uint16_t Step42_BuildPing(uint8_t *buf, uint16_t max_len, uint8_t seq)
{
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_PING, seq, 0, 0U);
}

uint16_t Step42_BuildEnable(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t enable)
{
    uint8_t payload[1];

    payload[0] = (enable != 0U) ? 1U : 0U;
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_ENABLE, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildSetAngle(uint8_t *buf, uint16_t max_len, uint8_t seq, float angle_deg)
{
    uint8_t payload[2];
    uint16_t angle_cdeg;

    if ((angle_deg < 0.0f) || (angle_deg > 360.0f))
    {
        return 0U;
    }

    angle_cdeg = Step42_FloatToU16Scaled(angle_deg, 100.0f);
    if (angle_cdeg > 36000U)
    {
        return 0U;
    }

    Step42_WriteU16(payload, 0U, angle_cdeg);
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_ANGLE, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildSetSpeed(uint8_t *buf,
                              uint16_t max_len,
                              uint8_t seq,
                              int16_t speed,
                              uint16_t accel)
{
    uint8_t payload[4];

    if ((speed < -5000) || (speed > 5000) || (accel == 0U))
    {
        return 0U;
    }

    Step42_WriteI16(payload, 0U, speed);
    Step42_WriteU16(payload, 2U, accel);
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_SPEED, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildMoveSteps(uint8_t *buf,
                               uint16_t max_len,
                               uint8_t seq,
                               int32_t steps,
                               uint16_t speed,
                               uint16_t accel)
{
    uint8_t payload[8];

    if ((steps == 0) || (speed == 0U) || (speed > 5000U) || (accel == 0U))
    {
        return 0U;
    }

    Step42_WriteI32(payload, 0U, steps);
    Step42_WriteU16(payload, 4U, speed);
    Step42_WriteU16(payload, 6U, accel);
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_MOVE_STEPS, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildSetMicrosteps(uint8_t *buf, uint16_t max_len, uint8_t seq, uint16_t microsteps)
{
    uint8_t payload[2];

    switch (microsteps)
    {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
        break;
    default:
        return 0U;
    }

    Step42_WriteU16(payload, 0U, microsteps);
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_MICROSTEP, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildSetPID(uint8_t *buf,
                            uint16_t max_len,
                            uint8_t seq,
                            float kp,
                            float ki,
                            float kd,
                            uint16_t limit,
                            float deadband)
{
    uint8_t payload[10];
    int16_t kp_x100;
    int16_t ki_x100;
    int16_t kd_x100;
    uint16_t dead_x1000;

    if ((limit == 0U) || (deadband < 0.0f) || (deadband > 10.0f))
    {
        return 0U;
    }

    kp_x100 = Step42_FloatToI16Scaled(kp, 100.0f);
    ki_x100 = Step42_FloatToI16Scaled(ki, 100.0f);
    kd_x100 = Step42_FloatToI16Scaled(kd, 100.0f);
    dead_x1000 = Step42_FloatToU16Scaled(deadband, 1000.0f);

    Step42_WriteI16(payload, 0U, kp_x100);
    Step42_WriteI16(payload, 2U, ki_x100);
    Step42_WriteI16(payload, 4U, kd_x100);
    Step42_WriteU16(payload, 6U, limit);
    Step42_WriteU16(payload, 8U, dead_x1000);
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_PID, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildSetStream(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t enable)
{
    uint8_t payload[1];

    payload[0] = (enable != 0U) ? 1U : 0U;
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_STREAM, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildQuery(uint8_t *buf, uint16_t max_len, uint8_t seq)
{
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_QUERY, seq, 0, 0U);
}

uint16_t Step42_BuildSetMode(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t mode)
{
    uint8_t payload[1];

    if ((mode != STEP42_MODE_OPEN_SPEED) &&
        (mode != STEP42_MODE_OPEN_POSITION) &&
        (mode != STEP42_MODE_CLOSED_POSITION))
    {
        return 0U;
    }

    payload[0] = mode;
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_MODE, seq, payload, sizeof(payload));
}

uint16_t Step42_BuildStop(uint8_t *buf, uint16_t max_len, uint8_t seq)
{
    return Step42_BuildFrame(buf, max_len, STEP42_CMD_STOP, seq, 0, 0U);
}

int Step42_ResponseStatus(const Step42_Response *response, uint8_t *status)
{
    if ((response == 0) || (status == 0) || (response->len < 1U))
    {
        return -1;
    }

    *status = response->payload[0];
    return 0;
}

int Step42_DecodeQueryStatus(const Step42_Response *response, Step42_Status *status)
{
    if ((response == 0) ||
        (status == 0) ||
        (response->cmd != STEP42_CMD_QUERY) ||
        (response->len < 10U))
    {
        return -1;
    }

    status->status = response->payload[0];
    status->enabled = response->payload[1];
    status->stream_enabled = response->payload[2];
    status->mode = response->payload[3];
    status->raw_angle = Step42_ReadU16(response->payload, 4U);
    status->angle_cdeg = Step42_ReadU16(response->payload, 6U);
    status->angle_deg = ((float)status->angle_cdeg) / 100.0f;
    status->pid_output = Step42_ReadI16(response->payload, 8U);
    if (response->len >= 20U)
    {
        status->kp = ((float)Step42_ReadI16(response->payload, 10U)) / 100.0f;
        status->ki = ((float)Step42_ReadI16(response->payload, 12U)) / 100.0f;
        status->kd = ((float)Step42_ReadI16(response->payload, 14U)) / 100.0f;
        status->pid_limit = Step42_ReadU16(response->payload, 16U);
        status->deadband = ((float)Step42_ReadU16(response->payload, 18U)) / 1000.0f;
    }
    if (response->len >= 22U)
    {
        status->microsteps = Step42_ReadU16(response->payload, 20U);
    }
    return 0;
}
