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
#define STEP42_MAX_PAYLOAD          (72U)
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
#define STEP42_CMD_MOVE_STEPS       (0x0AU)
#define STEP42_CMD_MICROSTEP        (0x0BU)
#define STEP42_CMD_CLOSED_ACCEL     (0x0CU)
#define STEP42_CMD_HOME_MODE        (0x0DU)
#define STEP42_CMD_SET_ZERO         (0x0EU)
#define STEP42_CMD_HOME             (0x0FU)
#define STEP42_CMD_TRACK_CONFIG     (0x10U)
#define STEP42_CMD_TRACK_PID        (0x11U)
#define STEP42_CMD_TRACK_ADVANCED   (0x12U)
#define STEP42_CMD_TRACK_INPUT      (0x13U)
#define STEP42_CMD_TRACK_DEFAULT    (0x14U)
#define STEP42_CMD_TRACK_QUERY      (0x15U)
#define STEP42_CMD_CHOPPER_MODE     (0x16U)
#define STEP42_CMD_RELATIVE_ANGLE   (0x17U)
#define STEP42_CMD_PROGRAM_MODE     (0x18U)
#define STEP42_CMD_PROGRAM_STEP     (0x19U)
#define STEP42_CMD_PROGRAM_SAVE     (0x1AU)
#define STEP42_CMD_PROGRAM_RUN      (0x1BU)
#define STEP42_CMD_PROGRAM_QUERY    (0x1CU)
#define STEP42_CMD_PROGRAM_STAGE    (0x1DU)
#define STEP42_CMD_MOTION_PROFILE   (0x1EU)
#define STEP42_CMD_TRACK_DIRECTION  (0x1FU)
#define STEP42_CMD_SAVE_RUNTIME_CONFIG (0x20U)
#define STEP42_CMD_CAN_NODE_ID      (0x21U)
#define STEP42_CMD_CURRENT          (0x26U)

#define STEP42_PROFILE_TRAPEZOID    (0x00U)
#define STEP42_PROFILE_S_CURVE      (0x01U)

#define STEP42_PROGRAM_OP_POSITION_ABS (0x01U)
#define STEP42_PROGRAM_OP_POSITION_REL (0x02U)
#define STEP42_PROGRAM_OP_OPEN_STEPS   (0x03U)
#define STEP42_PROGRAM_OP_SPEED        (0x04U)
#define STEP42_PROGRAM_OP_DELAY        (0x05U)
#define STEP42_PROGRAM_OP_HOME         (0x06U)
#define STEP42_PROGRAM_OP_TRACK        (0x07U)
#define STEP42_PROGRAM_OP_STOP         (0x08U)
#define STEP42_PROGRAM_OP_MICROSTEP    (0x09U)
#define STEP42_PROGRAM_OP_CHOPPER      (0x0AU)
#define STEP42_PROGRAM_SPEED_FLAG_RPM  (0x01U)

#define STEP42_STATUS_OK            (0x00U)
#define STEP42_STATUS_BAD_LEN       (0x01U)
#define STEP42_STATUS_RANGE         (0x02U)
#define STEP42_STATUS_UNKNOWN_CMD   (0x03U)
#define STEP42_STATUS_BAD_MODE      (0x04U)
#define STEP42_STATUS_BUSY          (0x05U)
#define STEP42_STATUS_DENIED        (0x06U)

#define STEP42_MODE_OPEN_POSITION   (0x00U)
#define STEP42_MODE_CLOSED_POSITION (0x01U)
#define STEP42_MODE_OPEN_SPEED      (0x02U)
#define STEP42_MODE_TRACK_PID       (0x03U)

#define STEP42_BOARD_FACTORY_DEBUG  (0x00U)
#define STEP42_BOARD_PWM            (0x01U)
#define STEP42_BOARD_CAN            (0x02U)
#define STEP42_BOARD_UART           (0x03U)

#define STEP42_HOME_NEAREST         (0x00U)
#define STEP42_HOME_POSITIVE        (0x01U)
#define STEP42_HOME_NEGATIVE        (0x02U)
#define STEP42_HOME_MULTI_TURN      (0x03U)

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
    float kp;
    float ki;
    float kd;
    uint16_t pid_limit;
    float deadband;
    uint16_t microsteps;
    uint16_t closed_accel;
    uint8_t zero_valid;
    uint8_t home_mode;
    uint8_t board_mode;
    uint8_t sensor_flags;
    float temperature_c;
    float speed_rpm;
    uint8_t tracking_mode;
    uint8_t tracking_frequency_khz;
    uint8_t tracking_speed_percent;
    uint8_t tracking_accel_percent;
    float tracking_primary_error;
    float tracking_secondary_feedback;
    int16_t tracking_output;
    uint8_t chopper_mode;
    float position_target_deg;
    float position_error_deg;
    float multi_turn_angle_deg;
    uint8_t position_pid_active;
    uint8_t response_interface;
    uint8_t program_mode;
    uint8_t program_running;
    uint8_t program_count;
    uint8_t motion_profile;
    uint8_t tracking_reverse;
    uint8_t can_node_id;
    uint8_t hold_current;
    uint8_t run_current;
} Step42_Status;

typedef struct
{
    uint8_t status;
    float current_angle_deg;
    float multi_turn_angle_deg;
    float target_angle_deg;
    float position_error_deg;
    uint8_t position_pid_active;
    float speed_rpm;
    uint16_t raw_angle;
    float absolute_angle_deg;
    float tracking_primary_error;
    float tracking_secondary_feedback;
    int16_t tracking_output;
    uint8_t tracking_mode;
} Step42_Telemetry;

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
uint16_t Step42_BuildMoveRelativeAngle(uint8_t *buf,
                                       uint16_t max_len,
                                       uint8_t seq,
                                       float delta_deg);
uint16_t Step42_BuildSetSpeed(uint8_t *buf,
                              uint16_t max_len,
                              uint8_t seq,
                              int16_t speed,
                              uint16_t accel);
uint16_t Step42_BuildMoveSteps(uint8_t *buf,
                               uint16_t max_len,
                               uint8_t seq,
                               int32_t steps,
                               uint16_t speed,
                               uint16_t accel);
uint16_t Step42_BuildSetMicrosteps(uint8_t *buf, uint16_t max_len, uint8_t seq, uint16_t microsteps);
uint16_t Step42_BuildSetClosedAccel(uint8_t *buf, uint16_t max_len, uint8_t seq, uint16_t accel);
uint16_t Step42_BuildSetHomeMode(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t mode);
uint16_t Step42_BuildSetZero(uint8_t *buf, uint16_t max_len, uint8_t seq);
uint16_t Step42_BuildHome(uint8_t *buf, uint16_t max_len, uint8_t seq, uint8_t mode);
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
uint16_t Step42_BuildSaveRuntimeConfig(uint8_t *buf, uint16_t max_len, uint8_t seq);
uint16_t Step42_BuildTrackConfig(uint8_t *buf, uint16_t max_len, uint8_t seq,
                                uint8_t mode, uint8_t speed_percent,
                                uint8_t accel_percent, uint8_t frequency_khz);
uint16_t Step42_BuildTrackPid(uint8_t *buf, uint16_t max_len, uint8_t seq,
                             uint8_t loop, float kp, float ki, float kd);
uint16_t Step42_BuildTrackAdvanced(uint8_t *buf, uint16_t max_len, uint8_t seq,
                                  float filter_alpha, float gain_min,
                                  float gain_max, float full_error,
                                  uint16_t prediction_ms, uint16_t timeout_ms,
                                  float single_deadband, float outer_deadband,
                                  float inner_deadband);
uint16_t Step42_BuildTrackInput(uint8_t *buf, uint16_t max_len, uint8_t seq,
                               float primary_error, float secondary_feedback);
uint16_t Step42_BuildTrackDefaults(uint8_t *buf, uint16_t max_len, uint8_t seq);
uint16_t Step42_BuildTrackQuery(uint8_t *buf, uint16_t max_len, uint8_t seq);
uint16_t Step42_BuildSetChopperMode(uint8_t *buf, uint16_t max_len,
                                   uint8_t seq, uint8_t mode);
uint16_t Step42_BuildSetMotionProfile(uint8_t *buf, uint16_t max_len,
                                      uint8_t seq, uint8_t profile);
uint16_t Step42_BuildSetTrackingDirection(uint8_t *buf, uint16_t max_len,
                                          uint8_t seq, uint8_t reverse);
uint16_t Step42_BuildSetCanNodeId(uint8_t *buf, uint16_t max_len,
                                  uint8_t seq, uint8_t node_id);
uint16_t Step42_BuildSetCurrent(uint8_t *buf, uint16_t max_len,
                               uint8_t seq, uint8_t hold_current,
                               uint8_t run_current);

int Step42_ResponseStatus(const Step42_Response *response, uint8_t *status);
int Step42_DecodeQueryStatus(const Step42_Response *response, Step42_Status *status);
int Step42_DecodeStreamTelemetry(const Step42_Response *response,
                                 Step42_Telemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif
