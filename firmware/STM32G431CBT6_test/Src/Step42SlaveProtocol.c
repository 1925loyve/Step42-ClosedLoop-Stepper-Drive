#include "Step42SlaveProtocol.h"

static uint8_t Step42Slave_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00U;
    uint8_t bit;

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

void Step42Slave_ParserInit(Step42Slave_Parser_t *parser)
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

uint8_t Step42Slave_ProcessByte(Step42Slave_Parser_t *parser,
                                uint8_t byte,
                                Step42Slave_Frame_t *frame)
{
    uint8_t crc_buf[CTRL_MAX_PAYLOAD + 3U];
    uint8_t crc;
    uint8_t i;

    if ((parser == 0) || (frame == 0))
    {
        return 0U;
    }

    switch (parser->state)
    {
    case 0:
        if (byte == CTRL_SOF1)
        {
            parser->state = 1U;
        }
        return 0U;

    case 1:
        if (byte == CTRL_SOF2)
        {
            parser->state = 2U;
        }
        else
        {
            parser->state = 0U;
        }
        return 0U;

    case 2:
        parser->len = byte;
        parser->index = 0U;
        if (parser->len > CTRL_MAX_PAYLOAD)
        {
            parser->state = 0U;
        }
        else
        {
            parser->state = 3U;
        }
        return 0U;

    case 3:
        parser->cmd = byte;
        parser->state = 4U;
        return 0U;

    case 4:
        parser->seq = byte;
        parser->state = (parser->len == 0U) ? 6U : 5U;
        return 0U;

    case 5:
        parser->payload[parser->index++] = byte;
        if (parser->index >= parser->len)
        {
            parser->state = 6U;
        }
        return 0U;

    case 6:
        crc_buf[0] = parser->len;
        crc_buf[1] = parser->cmd;
        crc_buf[2] = parser->seq;
        for (i = 0U; i < parser->len; i++)
        {
            crc_buf[3U + i] = parser->payload[i];
        }

        crc = Step42Slave_Crc8(crc_buf, (uint8_t)(parser->len + 3U));
        if (crc == byte)
        {
            frame->cmd = parser->cmd;
            frame->seq = parser->seq;
            frame->len = parser->len;
            for (i = 0U; i < parser->len; i++)
            {
                frame->payload[i] = parser->payload[i];
            }
            parser->state = 0U;
            return 1U;
        }
        parser->state = 0U;
        return 0U;

    default:
        parser->state = 0U;
        return 0U;
    }
}

uint8_t Step42Slave_BuildResponse(uint8_t *frame,
                                  uint8_t max_len,
                                  uint8_t cmd,
                                  uint8_t seq,
                                  const uint8_t *payload,
                                  uint8_t payload_len)
{
    uint8_t crc_buf[CTRL_MAX_PAYLOAD + 3U];
    uint8_t i;

    if ((frame == 0) ||
        (payload_len > CTRL_MAX_PAYLOAD) ||
        (max_len < (uint8_t)(payload_len + 6U)) ||
        ((payload_len > 0U) && (payload == 0)))
    {
        return 0U;
    }

    frame[0] = CTRL_RESP_SOF1;
    frame[1] = CTRL_RESP_SOF2;
    frame[2] = payload_len;
    frame[3] = (uint8_t)(cmd | 0x80U);
    frame[4] = seq;

    crc_buf[0] = payload_len;
    crc_buf[1] = frame[3];
    crc_buf[2] = seq;

    for (i = 0U; i < payload_len; i++)
    {
        frame[5U + i] = payload[i];
        crc_buf[3U + i] = payload[i];
    }

    frame[5U + payload_len] = Step42Slave_Crc8(crc_buf, (uint8_t)(payload_len + 3U));
    return (uint8_t)(payload_len + 6U);
}

int16_t Step42Slave_ReadI16(const uint8_t *payload)
{
    return (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
}

uint16_t Step42Slave_ReadU16(const uint8_t *payload)
{
    return (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
}

void Step42Slave_WriteU16(uint8_t *payload, uint8_t index, uint16_t value)
{
    payload[index] = (uint8_t)(value & 0xFFU);
    payload[index + 1U] = (uint8_t)(value >> 8);
}

void Step42Slave_WriteI16(uint8_t *payload, uint8_t index, int16_t value)
{
    Step42Slave_WriteU16(payload, index, (uint16_t)value);
}

