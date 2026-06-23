#ifndef STEP42_PROTOCOL_H
#define STEP42_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STEP42_SOF1                 (0xA5U)
#define STEP42_SOF2                 (0x5AU)
#define STEP42_RESP_SOF1            (0x5AU)
#define STEP42_RESP_SOF2            (0xA5U)
#define STEP42_MAX_PAYLOAD          (24U)
#define STEP42_MAX_FRAME_SIZE       (STEP42_MAX_PAYLOAD + 6U)

#define STEP42_CMD_PING             (0x01U)
#define STEP42_CMD_ENABLE           (0x02U)
#define STEP42_CMD_ANGLE            (0x03U)
#define STEP42_CMD_SPEED            (0x04U)
#define STEP42_CMD_PID              (0x05U)
#define STEP42_CMD_STREAM           (0x06U)
#define STEP42_CMD_QUERY            (0x07U)
#define STEP42_CMD_MODE             (0x08U)
#define STEP42_CMD_STOP             (0x09U)

#define STEP42_STATUS_OK            (0x00U)
#define STEP42_STATUS_BAD_LEN       (0x01U)
#define STEP42_STATUS_RANGE         (0x02U)
#define STEP42_STATUS_UNKNOWN_CMD   (0x03U)

#define STEP42_MODE_OPEN_POSITION   (0x00U)
#define STEP42_MODE_CLOSED_POSITION (0x01U)
#define STEP42_MODE_OPEN_SPEED      (0x02U)
#define STEP42_MODE_CLOSED_SPEED    (0x03U)

typedef enum
{
    STEP42_PARSE_NONE = 0,
    STEP42_PARSE_FRAME = 1,
    STEP42_PARSE_CRC_ERROR = -1,
    STEP42_PARSE_LENGTH_ERROR = -2
} Step42_ParseResult;

typedef struct
{
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[STEP42_MAX_PAYLOAD];
} Step42_Response;

typedef struct
{
    uint8_t state;
    uint8_t len;
    uint8_t cmd;
    uint8_t seq;
    uint8_t index;
    uint8_t payload[STEP42_MAX_PAYLOAD];
} Step42_Parser;

typedef struct
{
    uint8_t status;
    uint8_t enabled;
    uint8_t stream_enabled;
    uint8_t mode;
    uint16_t raw_angle;
    uint16_t angle_cdeg;
    float angle_deg;
    int16_t pid_output;
} Step42_Status;

uint8_t Step42_Crc8(const uint8_t *data, uint8_t len);
void Step42_ParserInit(Step42_Parser *parser);
Step42_ParseResult Step42_ParseByte(Step42_Parser *parser,
                                    uint8_t byte,
                                    Step42_Response *response);

uint16_t Step42_BuildFrame(uint8_t *buf,
                           uint16_t max_len,
                           uint8_t cmd,
                           uint8_t seq,
                           const uint8_t *payload,
                           uint8_t payload_len);

uint16_t Step42_BuildPing(uint8_t *buf, uint16_t max_len, uint8_t seq);
uint16_t Step42_BuildEnable(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t enable);
uint16_t Step42_BuildSetAngle(uint8_t *buf, uint16_t max_len, uint8_t seq, float angle_deg);
uint16_t Step42_BuildSetSpeed(uint8_t *buf,
                              uint16_t max_len,
                              uint8_t seq,
                              int16_t speed,
                              uint16_t accel);
uint16_t Step42_BuildSetPID(uint8_t *buf,
                            uint16_t max_len,
                            uint8_t seq,
                            float kp,
                            float ki,
                            float kd,
                            uint16_t limit,
                            float deadband);
uint16_t Step42_BuildSetStream(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t enable);
uint16_t Step42_BuildQuery(uint8_t *buf, uint16_t max_len, uint8_t seq);
uint16_t Step42_BuildSetMode(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t mode);
uint16_t Step42_BuildStop(uint8_t *buf, uint16_t max_len, uint8_t seq);

int Step42_ResponseStatus(const Step42_Response *response, uint8_t *status);
int Step42_DecodeQueryStatus(const Step42_Response *response, Step42_Status *status);

#ifdef __cplusplus
}
#endif

#endif
