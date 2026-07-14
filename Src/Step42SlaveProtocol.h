#ifndef STEP42_SLAVE_PROTOCOL_H
#define STEP42_SLAVE_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#define CTRL_SOF1 (0xA5U)
#define CTRL_SOF2 (0x5AU)
#define CTRL_RESP_SOF1 (0x5AU)
#define CTRL_RESP_SOF2 (0xA5U)
#define CTRL_MAX_PAYLOAD (64U)
#define CTRL_MAX_FRAME_SIZE (CTRL_MAX_PAYLOAD + 6U)
#define CTRL_PROTO_VERSION (9U)

#define CTRL_CMD_PING (0x01U)
#define CTRL_CMD_ENABLE (0x02U)
#define CTRL_CMD_ANGLE (0x03U)
#define CTRL_CMD_SPEED (0x04U)
#define CTRL_CMD_PID (0x05U)
#define CTRL_CMD_STREAM (0x06U)
#define CTRL_CMD_QUERY (0x07U)
#define CTRL_CMD_MODE (0x08U)
#define CTRL_CMD_STOP (0x09U)
#define CTRL_CMD_MOVE_STEPS (0x0AU)
#define CTRL_CMD_MICROSTEP (0x0BU)
#define CTRL_CMD_CLOSED_ACCEL (0x0CU)
#define CTRL_CMD_HOME_MODE (0x0DU)
#define CTRL_CMD_SET_ZERO (0x0EU)
#define CTRL_CMD_HOME (0x0FU)
#define CTRL_CMD_TRACK_CONFIG (0x10U)
#define CTRL_CMD_TRACK_PID (0x11U)
#define CTRL_CMD_TRACK_ADVANCED (0x12U)
#define CTRL_CMD_TRACK_INPUT (0x13U)
#define CTRL_CMD_TRACK_DEFAULT (0x14U)
#define CTRL_CMD_TRACK_QUERY (0x15U)
#define CTRL_CMD_CHOPPER_MODE (0x16U)
#define CTRL_CMD_RELATIVE_ANGLE (0x17U)
#define CTRL_CMD_PROGRAM_MODE (0x18U)
#define CTRL_CMD_PROGRAM_STEP (0x19U)
#define CTRL_CMD_PROGRAM_SAVE (0x1AU)
#define CTRL_CMD_PROGRAM_RUN (0x1BU)
#define CTRL_CMD_PROGRAM_QUERY (0x1CU)
#define CTRL_CMD_PROGRAM_STAGE (0x1DU)

#define CTRL_STATUS_OK (0x00U)
#define CTRL_STATUS_BAD_LEN (0x01U)
#define CTRL_STATUS_RANGE (0x02U)
#define CTRL_STATUS_UNKNOWN_CMD (0x03U)
#define CTRL_STATUS_BAD_MODE (0x04U)
#define CTRL_STATUS_BUSY (0x05U)
#define CTRL_STATUS_DENIED (0x06U)

#define CTRL_MODE_OPEN_POSITION (0x00U)
#define CTRL_MODE_CLOSED_POSITION (0x01U)
#define CTRL_MODE_OPEN_SPEED (0x02U)
#define CTRL_MODE_TRACK_PID (0x03U)

#define CTRL_HOME_NEAREST (0x00U)
#define CTRL_HOME_POSITIVE (0x01U)
#define CTRL_HOME_NEGATIVE (0x02U)
#define CTRL_HOME_MULTI_TURN (0x03U)

typedef struct
{
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[CTRL_MAX_PAYLOAD];
} Step42Slave_Frame_t;

typedef struct
{
    uint8_t state;
    uint8_t len;
    uint8_t cmd;
    uint8_t seq;
    uint8_t index;
    uint8_t payload[CTRL_MAX_PAYLOAD];
} Step42Slave_Parser_t;

void Step42Slave_ParserInit(Step42Slave_Parser_t *parser);
uint8_t Step42Slave_ProcessByte(Step42Slave_Parser_t *parser,
                                uint8_t byte,
                                Step42Slave_Frame_t *frame);
uint8_t Step42Slave_BuildResponse(uint8_t *frame,
                                  uint8_t max_len,
                                  uint8_t cmd,
                                  uint8_t seq,
                                  const uint8_t *payload,
                                  uint8_t payload_len);

int16_t Step42Slave_ReadI16(const uint8_t *payload);
uint16_t Step42Slave_ReadU16(const uint8_t *payload);
void Step42Slave_WriteU16(uint8_t *payload, uint8_t index, uint16_t value);
void Step42Slave_WriteI16(uint8_t *payload, uint8_t index, int16_t value);

#endif
