#include "Step42Comms.h"
#include "Step42Control.h"
#include "Step42SlaveProtocol.h"
#include "Step42BoardMode.h"
#include "Step42Sensors.h"
#include "Step42Tracking.h"
#include "Tmc2209.h"
#include <string.h>

#define STATUS_STREAM_INTERVAL_MS (50UL)
#define CTRL_RX_BUFFER_SIZE (256U)
#define CTRL_RX_BUFFER_MASK (CTRL_RX_BUFFER_SIZE - 1U)
#define VISION_ERROR_LINE_MAX (20U)
#define VISION_ERROR_ABS_LIMIT (10000.0f)
#define CTRL_INTERFACE_ERROR_UART (0U)
#define CTRL_INTERFACE_DEBUG_UART (1U)
#define CTRL_INTERFACE_CAN (2U)
#define STEP42_CAN_COMMAND_BASE (0x300U)
#define STEP42_CAN_RESPONSE_BASE (0x340U)
#define STEP42_CAN_NODE_ID_MASK (0x01FU)
#define STEP42_CAN_SEGMENT_START (0xA0U)
#define STEP42_CAN_SEGMENT_CONTINUE (0xB0U)
#define STEP42_CAN_REASSEMBLY_TIMEOUT_MS (30UL)
#define STEP42_CAN_TX_WAIT_MS (8UL)
#define CAN_RX_QUEUE_SIZE (16U)
#define CAN_RX_QUEUE_MASK (CAN_RX_QUEUE_SIZE - 1U)

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern FDCAN_HandleTypeDef hfdcan1;

typedef struct
{
  uint8_t len;
  uint8_t data[8];
} Step42CanPacket_t;

static Step42Slave_Parser_t error_parser;
static Step42Slave_Parser_t debug_parser;
static Step42Slave_Parser_t can_parser;
static UART_HandleTypeDef *response_uart = &huart2;
/* 0: Error UART (USART2), 1: Debug UART (USART3), 2: CAN. */
static uint8_t response_interface = CTRL_INTERFACE_ERROR_UART;
static UART_HandleTypeDef *stream_uart = &huart3;
static uint8_t stream_interface = CTRL_INTERFACE_DEBUG_UART;
static uint8_t error_rx_buffer[CTRL_RX_BUFFER_SIZE];
static uint8_t debug_rx_buffer[CTRL_RX_BUFFER_SIZE];
static volatile uint16_t error_rx_head = 0U;
static volatile uint16_t error_rx_tail = 0U;
static volatile uint16_t debug_rx_head = 0U;
static volatile uint16_t debug_rx_tail = 0U;
static uint8_t error_rx_byte = 0U;
static uint8_t debug_rx_byte = 0U;
static volatile uint8_t debug_tx_busy = 0U;
static volatile uint32_t debug_tx_pause_until_ms = 0U;
static volatile uint32_t debug_raw_last_tx_ms = 0U;
static volatile uint32_t debug_rx_count = 0U;
static uint8_t debug_stream_enable = 0U;
static char vision_error_line[VISION_ERROR_LINE_MAX];
static uint8_t vision_error_line_len = 0U;
static uint8_t can_started = 0U;
static Step42CanPacket_t can_rx_queue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t can_rx_head = 0U;
static volatile uint8_t can_rx_tail = 0U;
static uint8_t can_reassembly[CTRL_MAX_FRAME_SIZE];
static uint8_t can_reassembly_len = 0U;
static uint8_t can_reassembly_count = 0U;
static uint8_t can_reassembly_next_index = 0U;
static uint32_t can_reassembly_last_ms = 0U;

static void Ctrl_ProcessFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len);
static uint8_t Ctrl_CanSendRaw(const uint8_t *raw, uint8_t len);

static uint8_t Ctrl_CanSendPacket(const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef header;
  uint32_t start_ms;

  if ((can_started == 0U) || (len > 8U))
  {
    return 0U;
  }

  start_ms = HAL_GetTick();
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    if ((uint32_t)(HAL_GetTick() - start_ms) >= STEP42_CAN_TX_WAIT_MS)
    {
      return 0U;
    }
  }

  header.Identifier = STEP42_CAN_RESPONSE_BASE + Step42Control_GetCanNodeId();
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = len;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, (uint8_t *)data) == HAL_OK) ? 1U : 0U;
}

static uint8_t Ctrl_CanSendRaw(const uint8_t *raw, uint8_t len)
{
  uint8_t packet[8];
  uint8_t offset = 0U;
  uint8_t index = 0U;
  uint8_t chunk;

  if ((raw == 0) || (len == 0U) || (len > CTRL_MAX_FRAME_SIZE))
  {
    return 0U;
  }

  packet[0] = STEP42_CAN_SEGMENT_START;
  packet[1] = len;
  chunk = (len > 6U) ? 6U : len;
  memcpy(&packet[2], raw, chunk);
  if (Ctrl_CanSendPacket(packet, (uint8_t)(chunk + 2U)) == 0U)
  {
    return 0U;
  }
  offset = chunk;
  index = 1U;

  while (offset < len)
  {
    chunk = ((uint8_t)(len - offset) > 7U) ? 7U : (uint8_t)(len - offset);
    packet[0] = (uint8_t)(STEP42_CAN_SEGMENT_CONTINUE | index);
    memcpy(&packet[1], &raw[offset], chunk);
    if (Ctrl_CanSendPacket(packet, (uint8_t)(chunk + 1U)) == 0U)
    {
      return 0U;
    }
    offset = (uint8_t)(offset + chunk);
    index++;
  }

  return 1U;
}

static void Ctrl_SendBytes(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len)
{
  if (response_interface == CTRL_INTERFACE_CAN)
  {
    (void)Ctrl_CanSendRaw(buf, len);
    return;
  }

  if (huart == stream_uart)
  {
    debug_tx_busy = 1U;
  }
  HAL_UART_Transmit(huart, buf, len, 30);
  if (huart == stream_uart)
  {
    debug_tx_pause_until_ms = HAL_GetTick() + 50U;
    debug_tx_busy = 0U;
  }
}

static void Ctrl_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  uint8_t frame[CTRL_MAX_FRAME_SIZE];
  uint8_t frame_len;

  frame_len = Step42Slave_BuildResponse(frame, sizeof(frame), cmd, seq, payload, len);
  if (frame_len > 0U)
  {
    Ctrl_SendBytes(response_uart, frame, frame_len);
  }
}

static void Ctrl_SendStatus(uint8_t cmd, uint8_t seq, uint8_t status)
{
  uint8_t payload[1];

  payload[0] = status;
  Ctrl_SendFrame(cmd, seq, payload, sizeof(payload));
}

static int32_t Ctrl_ReadI32(const uint8_t *payload)
{
  return (int32_t)((uint32_t)payload[0] |
                   ((uint32_t)payload[1] << 8) |
                   ((uint32_t)payload[2] << 16) |
                   ((uint32_t)payload[3] << 24));
}

static void Ctrl_WriteI32(uint8_t *payload, uint8_t index, int32_t value)
{
  uint32_t raw = (uint32_t)value;

  payload[index] = (uint8_t)(raw & 0xFFU);
  payload[index + 1U] = (uint8_t)((raw >> 8) & 0xFFU);
  payload[index + 2U] = (uint8_t)((raw >> 16) & 0xFFU);
  payload[index + 3U] = (uint8_t)((raw >> 24) & 0xFFU);
}

static int16_t Ctrl_ClampI16(float value)
{
  if (value > 32767.0f)
  {
    return 32767;
  }
  if (value < -32768.0f)
  {
    return -32768;
  }
  return (int16_t)value;
}

static int32_t Ctrl_ClampI32(float value)
{
  if (value > 2147483647.0f)
  {
    return 2147483647;
  }
  if (value < -2147483648.0f)
  {
    return (-2147483647 - 1);
  }
  return (int32_t)value;
}

void Step42Comms_SendQuery(uint8_t seq)
{
  uint8_t resp[69];
  uint16_t angle_cdeg;
  int16_t pid_output;
  float kp;
  float ki;
  float kd;
  float pid_limit;
  float deadband;
  float angle_deg = Step42Control_GetAngleDeg();
  float pid_output_steps = Step42Control_GetPidOutputSteps();

  Step42Control_GetPositionPID(&kp, &ki, &kd, &pid_limit, &deadband);
  angle_cdeg = (uint16_t)(angle_deg * 100.0f + 0.5f);
  if (angle_cdeg >= 36000U)
  {
    angle_cdeg = 0U;
  }
  if (pid_output_steps > 32767.0f)
  {
    pid_output = 32767;
  }
  else if (pid_output_steps < -32768.0f)
  {
    pid_output = -32768;
  }
  else
  {
    pid_output = (int16_t)pid_output_steps;
  }

  resp[0] = CTRL_STATUS_OK;
  resp[1] = Step42Control_IsUserEnabled();
  resp[2] = debug_stream_enable;
  resp[3] = Step42Control_GetMode();
  Step42Slave_WriteU16(resp, 4U, Step42Control_GetRawAngle());
  Step42Slave_WriteU16(resp, 6U, angle_cdeg);
  Step42Slave_WriteI16(resp, 8U, pid_output);
  Ctrl_WriteI32(resp, 10U, (int32_t)(kp * 100.0f));
  Ctrl_WriteI32(resp, 14U, (int32_t)(ki * 100.0f));
  Ctrl_WriteI32(resp, 18U, (int32_t)(kd * 100.0f));
  Step42Slave_WriteU16(resp, 22U, (uint16_t)pid_limit);
  Step42Slave_WriteU16(resp, 24U, (uint16_t)(deadband * 1000.0f));
  Step42Slave_WriteU16(resp, 26U, Step42Control_GetMicrosteps());
  Step42Slave_WriteU16(resp, 28U, (uint16_t)Step42Control_GetClosedAccel());
  resp[30] = Step42Control_IsZeroValid();
  resp[31] = Step42Control_GetHomeMode();
  resp[32] = (uint8_t)Step42BoardMode_Get();
  resp[33] = Step42Sensors_GetFlags();
  Step42Slave_WriteI16(resp, 34U, Ctrl_ClampI16(Step42Sensors_GetTemperatureC() * 100.0f));
  Step42Slave_WriteI16(resp, 36U, Ctrl_ClampI16(Step42Control_GetSpeedRpm() * 10.0f));
  {
    const Step42TrackingConfig_t *tracking = Step42Control_GetTrackingConfig();
    resp[38] = tracking->mode;
    resp[39] = tracking->frequency_khz;
    resp[40] = tracking->speed_percent;
    resp[41] = tracking->accel_percent;
  }
  Step42Slave_WriteI16(resp, 42U, Ctrl_ClampI16(Step42Tracking_GetPrimaryError() * 100.0f));
  Step42Slave_WriteI16(resp, 44U, Ctrl_ClampI16(Step42Tracking_GetSecondaryFeedback() * 100.0f));
  Step42Slave_WriteI16(resp, 46U, Ctrl_ClampI16(Step42Tracking_GetOutput()));
  resp[48] = Tmc2209_GetChopperMode();
  Ctrl_WriteI32(resp, 49U,
                Ctrl_ClampI32(Step42Control_GetPositionTargetDeg() * 100.0f));
  Ctrl_WriteI32(resp, 53U,
                Ctrl_ClampI32(Step42Control_GetPositionErrorDeg() * 100.0f));
  resp[57] = Step42Control_IsPositionPidActive();
  resp[58] = response_interface;
  resp[59] = Step42Control_IsProgramMode();
  resp[60] = Step42Control_IsProgramRunning();
  resp[61] = Step42Control_GetProgramCount();
  Ctrl_WriteI32(resp, 62U,
                Ctrl_ClampI32(Step42Control_GetMultiTurnAngleDeg() * 100.0f));
  resp[66] = Step42Control_GetMotionProfile();
  resp[67] = Step42Control_GetTrackingDirection();
  resp[68] = Step42Control_GetCanNodeId();
  Ctrl_SendFrame(CTRL_CMD_QUERY, seq, resp, sizeof(resp));
}

static void Step42Comms_SendTelemetry(void)
{
  uint8_t resp[31];
  uint16_t angle_cdeg = (uint16_t)(Step42Control_GetAngleDeg() * 100.0f + 0.5f);

  if (angle_cdeg >= 36000U)
  {
    angle_cdeg = 0U;
  }
  resp[0] = CTRL_STATUS_OK;
  Step42Slave_WriteU16(resp, 1U, angle_cdeg);
  Ctrl_WriteI32(resp, 3U,
                Ctrl_ClampI32(Step42Control_GetPositionTargetDeg() * 100.0f));
  Ctrl_WriteI32(resp, 7U,
                Ctrl_ClampI32(Step42Control_GetPositionErrorDeg() * 100.0f));
  resp[11] = Step42Control_IsPositionPidActive();
  Step42Slave_WriteI16(resp, 12U,
                       Ctrl_ClampI16(Step42Control_GetSpeedRpm() * 10.0f));
  Step42Slave_WriteU16(resp, 14U, Step42Control_GetRawAngle());
  Ctrl_WriteI32(resp, 16U,
                Ctrl_ClampI32(Step42Tracking_GetPrimaryError() * 100.0f));
  Ctrl_WriteI32(resp, 20U,
                Ctrl_ClampI32(Step42Tracking_GetSecondaryFeedback() * 100.0f));
  Step42Slave_WriteI16(resp, 24U,
                       Ctrl_ClampI16(Step42Tracking_GetOutput()));
  resp[26] = Step42Control_GetTrackingConfig()->mode;
  Ctrl_WriteI32(resp, 27U,
                Ctrl_ClampI32(Step42Control_GetMultiTurnAngleDeg() * 100.0f));
  Ctrl_SendFrame(CTRL_CMD_STREAM, 0U, resp, sizeof(resp));
}

static void Step42Comms_SendTrackingQuery(uint8_t seq)
{
  uint8_t resp[60];
  const Step42TrackingConfig_t *cfg = Step42Control_GetTrackingConfig();

  resp[0] = CTRL_STATUS_OK;
  resp[1] = cfg->mode;
  resp[2] = cfg->speed_percent;
  resp[3] = cfg->accel_percent;
  resp[4] = cfg->frequency_khz;
  Ctrl_WriteI32(resp, 5U, (int32_t)(cfg->single.kp * 100.0f));
  Ctrl_WriteI32(resp, 9U, (int32_t)(cfg->single.ki * 100.0f));
  Ctrl_WriteI32(resp, 13U, (int32_t)(cfg->single.kd * 100.0f));
  Ctrl_WriteI32(resp, 17U, (int32_t)(cfg->outer.kp * 100.0f));
  Ctrl_WriteI32(resp, 21U, (int32_t)(cfg->outer.ki * 100.0f));
  Ctrl_WriteI32(resp, 25U, (int32_t)(cfg->outer.kd * 100.0f));
  Ctrl_WriteI32(resp, 29U, (int32_t)(cfg->inner.kp * 100.0f));
  Ctrl_WriteI32(resp, 33U, (int32_t)(cfg->inner.ki * 100.0f));
  Ctrl_WriteI32(resp, 37U, (int32_t)(cfg->inner.kd * 100.0f));
  Step42Slave_WriteU16(resp, 41U, (uint16_t)(cfg->filter_alpha * 1000.0f));
  Step42Slave_WriteU16(resp, 43U, (uint16_t)(cfg->dynamic_gain_min * 1000.0f));
  Step42Slave_WriteU16(resp, 45U, (uint16_t)(cfg->dynamic_gain_max * 1000.0f));
  Step42Slave_WriteU16(resp, 47U, (uint16_t)(cfg->dynamic_full_error * 100.0f));
  Step42Slave_WriteU16(resp, 49U, (uint16_t)cfg->prediction_ms);
  Step42Slave_WriteU16(resp, 51U, cfg->timeout_ms);
  Step42Slave_WriteU16(resp, 53U, (uint16_t)(cfg->single.deadband * 1000.0f));
  Step42Slave_WriteU16(resp, 55U, (uint16_t)(cfg->outer.deadband * 1000.0f));
  Step42Slave_WriteU16(resp, 57U, (uint16_t)(cfg->inner.deadband * 1000.0f));
  resp[59] = Step42Control_GetTrackingDirection();
  Ctrl_SendFrame(CTRL_CMD_TRACK_QUERY, seq, resp, sizeof(resp));
}

static void Step42Comms_SendExternalPositionQuery(uint8_t seq)
{
  uint8_t resp[44];
  const Step42ExternalPositionConfig_t *cfg =
      Step42Control_GetExternalPositionConfig();

  resp[0] = CTRL_STATUS_OK;
  Step42Slave_WriteU16(resp, 1U, cfg->microsteps);
  Step42Slave_WriteU16(resp, 3U, cfg->accel);
  Ctrl_WriteI32(resp, 5U, (int32_t)(cfg->kp * 100.0f));
  Ctrl_WriteI32(resp, 9U, (int32_t)(cfg->ki * 100.0f));
  Ctrl_WriteI32(resp, 13U, (int32_t)(cfg->kd * 100.0f));
  Step42Slave_WriteU16(resp, 17U, (uint16_t)cfg->output_limit);
  Step42Slave_WriteU16(resp, 19U, (uint16_t)(cfg->deadband * 1000.0f));
  Ctrl_WriteI32(resp, 21U, (int32_t)(cfg->minimum_deg * 100.0f));
  Ctrl_WriteI32(resp, 25U, (int32_t)(cfg->maximum_deg * 100.0f));
  Step42Slave_WriteU16(resp, 29U, cfg->timeout_ms);
  Ctrl_WriteI32(resp, 31U, (int32_t)(cfg->center_raw_deg * 100.0f));
  resp[35] = cfg->center_valid;
  Ctrl_WriteI32(resp, 36U,
                Ctrl_ClampI32(Step42Control_GetExternalPositionTargetDeg() * 100.0f));
  Ctrl_WriteI32(resp, 40U,
                Ctrl_ClampI32(Step42Control_GetExternalPositionCurrentDeg() * 100.0f));
  Ctrl_SendFrame(CTRL_CMD_EXTERNAL_POSITION_QUERY, seq, resp, sizeof(resp));
}

static uint8_t Ctrl_IsExternalRunCommand(uint8_t cmd)
{
  switch (cmd)
  {
  case CTRL_CMD_ANGLE:
  case CTRL_CMD_SPEED:
  case CTRL_CMD_RELATIVE_ANGLE:
  case CTRL_CMD_MODE:
  case CTRL_CMD_MOVE_STEPS:
  case CTRL_CMD_HOME:
  case CTRL_CMD_EXTERNAL_POSITION_TARGET:
    return 1U;
  default:
    return 0U;
  }
}

static uint8_t Ctrl_IsProgramModeAllowed(uint8_t cmd)
{
  switch (cmd)
  {
  case CTRL_CMD_PING:
  case CTRL_CMD_ENABLE:
  case CTRL_CMD_STREAM:
  case CTRL_CMD_QUERY:
  case CTRL_CMD_STOP:
  /* Tracking input is sensor data, not an external motion command. It must
     remain available while a program-owned tracking step is running. */
  case CTRL_CMD_TRACK_INPUT:
  case CTRL_CMD_TRACK_QUERY:
  case CTRL_CMD_EXTERNAL_POSITION_CONFIG:
  case CTRL_CMD_EXTERNAL_POSITION_QUERY:
  case CTRL_CMD_EXTERNAL_POSITION_CENTER:
  case CTRL_CMD_CAN_NODE_ID:
  case CTRL_CMD_PROGRAM_MODE:
  case CTRL_CMD_PROGRAM_STEP:
  case CTRL_CMD_PROGRAM_SAVE:
  case CTRL_CMD_PROGRAM_RUN:
  case CTRL_CMD_PROGRAM_QUERY:
  case CTRL_CMD_PROGRAM_STAGE:
    return 1U;
  default:
    return 0U;
  }
}

static void Ctrl_ProcessFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  if ((Step42Control_IsProgramMode() != 0U) &&
      (Ctrl_IsProgramModeAllowed(cmd) == 0U))
  {
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_DENIED);
    return;
  }
  if ((Step42Control_IsProgramRunning() != 0U) &&
      (Ctrl_IsExternalRunCommand(cmd) != 0U))
  {
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_DENIED);
    return;
  }

  if (cmd == CTRL_CMD_PING)
  {
    uint8_t resp[2];

    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    resp[0] = CTRL_STATUS_OK;
    resp[1] = CTRL_PROTO_VERSION;
    Ctrl_SendFrame(cmd, seq, resp, sizeof(resp));
    return;
  }

  if (cmd == CTRL_CMD_ENABLE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    Step42Control_SetUserEnable((payload[0] != 0U) ? 1U : 0U);
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_ANGLE)
  {
    int32_t target_cdeg;
    float target_angle;

    if ((len != 2U) && (len != 4U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    if (len == 2U)
    {
      target_cdeg = (int32_t)Step42Slave_ReadU16(payload);
    }
    else
    {
      target_cdeg = Ctrl_ReadI32(payload);
    }
    if ((target_cdeg < -720000) || (target_cdeg > 720000))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    if (Step42Control_GetMode() != CTRL_MODE_CLOSED_POSITION)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }

    target_angle = ((float)target_cdeg) / 100.0f;
    Step42Control_MoveToAngleByMode(target_angle);
    Step42Control_MarkExternalTask();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_SPEED)
  {
    int16_t speed;
    uint16_t accel;

    if (len != 4U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    speed = Step42Slave_ReadI16(payload);
    accel = Step42Slave_ReadU16(payload + 2U);
    if ((speed < -5000) || (speed > 5000) || (accel == 0U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    if (Step42Control_GetMode() != CTRL_MODE_OPEN_SPEED)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }

    Step42Control_RunSpeedByMode((float)speed, (float)accel);
    Step42Control_MarkExternalTask();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_RELATIVE_ANGLE)
  {
    int32_t delta_cdeg;

    if (len != 4U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    delta_cdeg = Ctrl_ReadI32(payload);
    if ((delta_cdeg < -720000) || (delta_cdeg > 720000))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    if (Step42Control_MoveRelativeAngleByMode(((float)delta_cdeg) / 100.0f) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    Step42Control_MarkExternalTask();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_PID)
  {
    int32_t kp_x100;
    int32_t ki_x100;
    int32_t kd_x100;
    uint16_t limit;
    uint16_t dead_x1000;

    if ((len != 10U) && (len != 16U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    if (len == 16U)
    {
      kp_x100 = Ctrl_ReadI32(payload);
      ki_x100 = Ctrl_ReadI32(payload + 4U);
      kd_x100 = Ctrl_ReadI32(payload + 8U);
      limit = Step42Slave_ReadU16(payload + 12U);
      dead_x1000 = Step42Slave_ReadU16(payload + 14U);
    }
    else
    {
      kp_x100 = Step42Slave_ReadI16(payload);
      ki_x100 = Step42Slave_ReadI16(payload + 2U);
      kd_x100 = Step42Slave_ReadI16(payload + 4U);
      limit = Step42Slave_ReadU16(payload + 6U);
      dead_x1000 = Step42Slave_ReadU16(payload + 8U);
    }
    if ((limit < 1U) || (dead_x1000 > 10000U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    Step42Control_SetPositionPID(((float)kp_x100) / 100.0f,
                                 ((float)ki_x100) / 100.0f,
                                 ((float)kd_x100) / 100.0f,
                                 (float)limit,
                                 ((float)dead_x1000) / 1000.0f);
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_STREAM)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (payload[0] != 0U)
    {
      stream_uart = response_uart;
      stream_interface = response_interface;
      debug_stream_enable = 1U;
    }
    else
    {
      debug_stream_enable = 0U;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_QUERY)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    Step42Comms_SendQuery(seq);
    return;
  }

  if (cmd == CTRL_CMD_MODE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if ((payload[0] != CTRL_MODE_OPEN_SPEED) &&
        (payload[0] != CTRL_MODE_OPEN_POSITION) &&
        (payload[0] != CTRL_MODE_CLOSED_POSITION) &&
        (payload[0] != CTRL_MODE_TRACK_PID) &&
        (payload[0] != CTRL_MODE_EXTERNAL_POSITION))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    if (Step42Control_SetMode(payload[0]) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_STOP)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    Step42Control_AbortProgram();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_MOVE_STEPS)
  {
    int32_t steps;
    uint16_t speed;
    uint16_t accel;
    uint8_t mode;

    if (len != 8U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    steps = Ctrl_ReadI32(payload);
    speed = Step42Slave_ReadU16(payload + 4U);
    accel = Step42Slave_ReadU16(payload + 6U);
    if ((steps == 0) || (speed == 0U) || (speed > 5000U) || (accel == 0U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    mode = Step42Control_GetMode();
    if (mode != CTRL_MODE_OPEN_POSITION)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }

    if (Step42Control_MoveStepsByMode(steps, (float)speed, (float)accel) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    Step42Control_MarkExternalTask();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_MICROSTEP)
  {
    uint16_t microsteps;

    if (len != 2U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    microsteps = Step42Slave_ReadU16(payload);
    if (Step42Control_GetMode() == CTRL_MODE_TRACK_PID)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }
    if (Step42Control_SetMicrosteps(microsteps) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_CLOSED_ACCEL)
  {
    uint16_t accel;

    if (len != 2U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    accel = Step42Slave_ReadU16(payload);
    if (Step42Control_SetClosedAccel((float)accel) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_HOME_MODE)
  {
    uint8_t status;

    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    status = Step42Control_SetHomeMode(payload[0]);
    Ctrl_SendStatus(cmd, seq, status);
    return;
  }

  if (cmd == CTRL_CMD_SET_ZERO)
  {
    uint8_t status;

    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    status = Step42Control_SetCurrentZero();
    Ctrl_SendStatus(cmd, seq, status);
    return;
  }

  if (cmd == CTRL_CMD_HOME)
  {
    uint8_t mode;
    uint8_t status;

    if (len > 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    mode = (len == 1U) ? payload[0] : 0xFFU;
    status = Step42Control_Home(mode);
    if (status == CTRL_STATUS_OK)
    {
      Step42Control_MarkExternalTask();
    }
    Ctrl_SendStatus(cmd, seq, status);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_CONFIG)
  {
    if (len != 4U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_SetTrackingBasic(payload[0], payload[1], payload[2], payload[3]) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_PID)
  {
    if (len != 13U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_SetTrackingPid(payload[0],
                                     ((float)Ctrl_ReadI32(payload + 1U)) / 100.0f,
                                     ((float)Ctrl_ReadI32(payload + 5U)) / 100.0f,
                                     ((float)Ctrl_ReadI32(payload + 9U)) / 100.0f) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_ADVANCED)
  {
    if (len != 18U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_SetTrackingAdvanced(
            ((float)Step42Slave_ReadU16(payload)) / 1000.0f,
            ((float)Step42Slave_ReadU16(payload + 2U)) / 1000.0f,
            ((float)Step42Slave_ReadU16(payload + 4U)) / 1000.0f,
            ((float)Step42Slave_ReadU16(payload + 6U)) / 100.0f,
            (float)Step42Slave_ReadU16(payload + 8U),
            Step42Slave_ReadU16(payload + 10U),
            ((float)Step42Slave_ReadU16(payload + 12U)) / 1000.0f,
            ((float)Step42Slave_ReadU16(payload + 14U)) / 1000.0f,
            ((float)Step42Slave_ReadU16(payload + 16U)) / 1000.0f) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_INPUT)
  {
    if ((len != 4U) && (len != 8U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_GetMode() != CTRL_MODE_TRACK_PID)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }
    if (len == 8U)
    {
      int32_t primary = Ctrl_ReadI32(payload);
      int32_t secondary = Ctrl_ReadI32(payload + 4U);
      if ((primary < -100000) || (primary > 100000) ||
          (secondary < -100000) || (secondary > 100000))
      {
        Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
        return;
      }
      Step42Control_SetTrackingInput(((float)primary) / 100.0f,
                                     ((float)secondary) / 100.0f);
    }
    else
    {
      Step42Control_SetTrackingInput(((float)Step42Slave_ReadI16(payload)) / 100.0f,
                                     ((float)Step42Slave_ReadI16(payload + 2U)) / 100.0f);
    }
    Step42Control_MarkExternalTask();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_DEFAULT)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Step42Control_ResetTrackingDefaults();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_QUERY)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Step42Comms_SendTrackingQuery(seq);
    return;
  }

  if (cmd == CTRL_CMD_EXTERNAL_POSITION_CONFIG)
  {
    Step42ExternalPositionConfig_t cfg;

    if (len != 30U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    cfg = *Step42Control_GetExternalPositionConfig();
    cfg.microsteps = Step42Slave_ReadU16(payload);
    cfg.accel = Step42Slave_ReadU16(payload + 2U);
    cfg.kp = ((float)Ctrl_ReadI32(payload + 4U)) / 100.0f;
    cfg.ki = ((float)Ctrl_ReadI32(payload + 8U)) / 100.0f;
    cfg.kd = ((float)Ctrl_ReadI32(payload + 12U)) / 100.0f;
    cfg.output_limit = (float)Step42Slave_ReadU16(payload + 16U);
    cfg.deadband = ((float)Step42Slave_ReadU16(payload + 18U)) / 1000.0f;
    cfg.minimum_deg = ((float)Ctrl_ReadI32(payload + 20U)) / 100.0f;
    cfg.maximum_deg = ((float)Ctrl_ReadI32(payload + 24U)) / 100.0f;
    cfg.timeout_ms = Step42Slave_ReadU16(payload + 28U);
    if (Step42Control_SetExternalPositionConfig(&cfg) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_EXTERNAL_POSITION_TARGET)
  {
    int32_t target_cdeg;

    if (len != 4U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_GetMode() != CTRL_MODE_EXTERNAL_POSITION)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }
    target_cdeg = Ctrl_ReadI32(payload);
    if ((target_cdeg < -720000) || (target_cdeg > 720000) ||
        (Step42Control_SetExternalPositionTarget(((float)target_cdeg) / 100.0f) == 0U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Step42Control_MarkExternalTask();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_EXTERNAL_POSITION_QUERY)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Step42Comms_SendExternalPositionQuery(seq);
    return;
  }

  if (cmd == CTRL_CMD_EXTERNAL_POSITION_CENTER)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq,
                    (Step42Control_SetExternalPositionCenter() != 0U) ?
                    CTRL_STATUS_OK : CTRL_STATUS_RANGE);
    return;
  }

  if (cmd == CTRL_CMD_CHOPPER_MODE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Tmc2209_SetChopperMode(payload[0]) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_MOTION_PROFILE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_SetMotionProfile(payload[0]) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_TRACK_DIRECTION)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (payload[0] > 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }
    Step42Control_SetTrackingDirection(payload[0]);
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_SAVE_RUNTIME_CONFIG)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq, Step42Control_SaveRuntimeConfig());
    return;
  }

  if (cmd == CTRL_CMD_CAN_NODE_ID)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq, Step42Control_SetCanNodeId(payload[0]));
    return;
  }

  if (cmd == CTRL_CMD_PROGRAM_MODE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq, Step42Control_SetProgramMode(payload[0]));
    return;
  }

  if (cmd == CTRL_CMD_PROGRAM_STEP)
  {
    Step42ProgramStep_t step;

    if (len != 13U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    step.op = payload[1];
    step.flags = payload[2];
    step.speed = Step42Slave_ReadU16(payload + 3U);
    step.accel = Step42Slave_ReadU16(payload + 5U);
    step.duration_ms = Step42Slave_ReadU16(payload + 7U);
    step.value = Ctrl_ReadI32(payload + 9U);
    Ctrl_SendStatus(cmd, seq, Step42Control_SetProgramStep(payload[0], &step));
    return;
  }

  if (cmd == CTRL_CMD_PROGRAM_SAVE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq, Step42Control_SaveProgram(payload[0]));
    return;
  }

  if (cmd == CTRL_CMD_PROGRAM_STAGE)
  {
    if (len != 1U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq, Step42Control_StageProgram(payload[0]));
    return;
  }

  if (cmd == CTRL_CMD_PROGRAM_RUN)
  {
    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    Ctrl_SendStatus(cmd, seq, Step42Control_StartProgram());
    return;
  }

  if (cmd == CTRL_CMD_PROGRAM_QUERY)
  {
    uint8_t resp[4];

    if (len != 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    resp[0] = CTRL_STATUS_OK;
    resp[1] = Step42Control_IsProgramMode();
    resp[2] = Step42Control_IsProgramRunning();
    resp[3] = Step42Control_GetProgramCount();
    Ctrl_SendFrame(cmd, seq, resp, sizeof(resp));
    return;
  }

  Ctrl_SendStatus(cmd, seq, CTRL_STATUS_UNKNOWN_CMD);
}

static uint8_t Ctrl_DebugCommandAllowed(uint8_t cmd)
{
  switch (cmd)
  {
  case CTRL_CMD_PING:
  case CTRL_CMD_ENABLE:
  case CTRL_CMD_PID:
  case CTRL_CMD_STREAM:
  case CTRL_CMD_QUERY:
  case CTRL_CMD_MODE:
  case CTRL_CMD_STOP:
  case CTRL_CMD_TRACK_CONFIG:
  case CTRL_CMD_TRACK_PID:
  case CTRL_CMD_TRACK_ADVANCED:
  case CTRL_CMD_TRACK_INPUT:
  case CTRL_CMD_TRACK_DEFAULT:
  case CTRL_CMD_TRACK_QUERY:
  case CTRL_CMD_CHOPPER_MODE:
  case CTRL_CMD_MOTION_PROFILE:
  case CTRL_CMD_TRACK_DIRECTION:
  case CTRL_CMD_SAVE_RUNTIME_CONFIG:
  case CTRL_CMD_CAN_NODE_ID:
  case CTRL_CMD_EXTERNAL_POSITION_CONFIG:
  case CTRL_CMD_EXTERNAL_POSITION_QUERY:
  case CTRL_CMD_EXTERNAL_POSITION_CENTER:
  case CTRL_CMD_PROGRAM_MODE:
  case CTRL_CMD_PROGRAM_STEP:
  case CTRL_CMD_PROGRAM_SAVE:
  case CTRL_CMD_PROGRAM_RUN:
  case CTRL_CMD_PROGRAM_QUERY:
  case CTRL_CMD_PROGRAM_STAGE:
    return 1U;
  default:
    return 0U;
  }
}

static uint8_t Ctrl_CanDebugCommandAllowed(uint8_t cmd)
{
  switch (cmd)
  {
  case CTRL_CMD_PING:
  case CTRL_CMD_PID:
  case CTRL_CMD_STREAM:
  case CTRL_CMD_QUERY:
  case CTRL_CMD_TRACK_CONFIG:
  case CTRL_CMD_TRACK_PID:
  case CTRL_CMD_TRACK_ADVANCED:
  case CTRL_CMD_TRACK_DEFAULT:
  case CTRL_CMD_TRACK_QUERY:
  case CTRL_CMD_TRACK_DIRECTION:
  case CTRL_CMD_SAVE_RUNTIME_CONFIG:
  case CTRL_CMD_CAN_NODE_ID:
  case CTRL_CMD_EXTERNAL_POSITION_CONFIG:
  case CTRL_CMD_EXTERNAL_POSITION_TARGET:
  case CTRL_CMD_EXTERNAL_POSITION_QUERY:
  case CTRL_CMD_EXTERNAL_POSITION_CENTER:
    return 1U;
  default:
    return 0U;
  }
}

static uint8_t Ctrl_ParseVisionError(const char *text, uint8_t len, float *error)
{
  uint8_t index = 0U;
  uint8_t digit_seen = 0U;
  float value = 0.0f;
  float sign = 1.0f;

  if ((text == 0) || (error == 0) || (len == 0U))
  {
    return 0U;
  }
  if ((text[index] == '+') || (text[index] == '-'))
  {
    sign = (text[index] == '-') ? -1.0f : 1.0f;
    index++;
  }
  for (; index < len; index++)
  {
    char ch = text[index];

    if ((ch >= '0') && (ch <= '9'))
    {
      digit_seen = 1U;
      value = value * 10.0f + (float)(ch - '0');
    }
    else
    {
      return 0U;
    }
  }
  value *= sign;
  if ((digit_seen == 0U) || (value > VISION_ERROR_ABS_LIMIT) ||
      (value < -VISION_ERROR_ABS_LIMIT))
  {
    return 0U;
  }
  *error = value;
  return 1U;
}

static void Ctrl_ProcessVisionErrorByte(uint8_t value)
{
  float error;

  if (value == '\r')
  {
    return;
  }
  if (value == '\n')
  {
    if ((Step42Control_GetMode() == CTRL_MODE_TRACK_PID) &&
        (Ctrl_ParseVisionError(vision_error_line, vision_error_line_len, &error) != 0U))
    {
      Step42Control_SetTrackingInput(error, 0.0f);
      Step42Control_MarkExternalTask();
    }
    vision_error_line_len = 0U;
    return;
  }
  if (((value >= '0') && (value <= '9')) || (value == '+') ||
      (value == '-'))
  {
    if (vision_error_line_len < (VISION_ERROR_LINE_MAX - 1U))
    {
      vision_error_line[vision_error_line_len++] = (char)value;
    }
    else
    {
      vision_error_line_len = 0U;
    }
  }
  else
  {
    vision_error_line_len = 0U;
  }
}

static void Ctrl_ProcessRxByte(Step42Slave_Parser_t *parser,
                               uint8_t ch,
                               UART_HandleTypeDef *huart,
                               uint8_t restricted)
{
  Step42Slave_Frame_t frame;

  /* USART2 accepts newline-delimited visual error only while idle. Binary
     control frames keep their original A5 5A parser and are never mixed. */
  if ((huart == &huart2) &&
      (Step42BoardMode_Get() == STEP42_BOARD_UART) &&
      (parser->state == 0U) && (ch != CTRL_SOF1))
  {
    Ctrl_ProcessVisionErrorByte(ch);
  }

  if (Step42Slave_ProcessByte(parser, ch, &frame) != 0U)
  {
    /* Bind a reply to the UART that completed this frame, not the last
       receiver buffer scanned by the main loop. */
    response_uart = huart;
    response_interface = (huart == &huart3) ?
                         CTRL_INTERFACE_DEBUG_UART : CTRL_INTERFACE_ERROR_UART;
    if (((restricted == 1U) && (Ctrl_DebugCommandAllowed(frame.cmd) == 0U)) ||
        ((restricted == 2U) && (Ctrl_CanDebugCommandAllowed(frame.cmd) == 0U)))
    {
      Ctrl_SendStatus(frame.cmd, frame.seq, CTRL_STATUS_DENIED);
      return;
    }
    Ctrl_ProcessFrame(frame.cmd, frame.seq, frame.payload, frame.len);
  }
}

static void Ctrl_RxPush(uint8_t *buffer,
                        volatile uint16_t *head,
                        volatile uint16_t *tail,
                        uint8_t value)
{
  uint16_t next = (uint16_t)((*head + 1U) & CTRL_RX_BUFFER_MASK);

  if (next == *tail)
  {
    *tail = (uint16_t)((*tail + 1U) & CTRL_RX_BUFFER_MASK);
  }
  buffer[*head] = value;
  *head = next;
}

static void Ctrl_ProcessBuffer(uint8_t *buffer,
                               volatile uint16_t *head,
                               volatile uint16_t *tail,
                               Step42Slave_Parser_t *parser,
                               UART_HandleTypeDef *huart,
                               uint8_t restricted)
{
  while (*tail != *head)
  {
    uint8_t value = buffer[*tail];
    *tail = (uint16_t)((*tail + 1U) & CTRL_RX_BUFFER_MASK);
    debug_rx_count++;
    Ctrl_ProcessRxByte(parser, value, huart, restricted);
  }
}

static void Ctrl_CanResetReassembly(void)
{
  can_reassembly_len = 0U;
  can_reassembly_count = 0U;
  can_reassembly_next_index = 0U;
  can_reassembly_last_ms = 0U;
}

static void Ctrl_CanPushPacket(const uint8_t *data, uint8_t len)
{
  uint8_t next = (uint8_t)((can_rx_head + 1U) & CAN_RX_QUEUE_MASK);

  if ((data == 0) || (len == 0U) || (len > 8U))
  {
    return;
  }
  if (next == can_rx_tail)
  {
    can_rx_tail = (uint8_t)((can_rx_tail + 1U) & CAN_RX_QUEUE_MASK);
  }
  can_rx_queue[can_rx_head].len = len;
  memcpy(can_rx_queue[can_rx_head].data, data, len);
  can_rx_head = next;
}

static void Ctrl_CanProcessCompleteFrame(void)
{
  Step42Slave_Frame_t frame;
  uint8_t index;

  Step42Slave_ParserInit(&can_parser);
  for (index = 0U; index < can_reassembly_len; index++)
  {
    if (Step42Slave_ProcessByte(&can_parser, can_reassembly[index], &frame) != 0U)
    {
      response_uart = 0;
      response_interface = CTRL_INTERFACE_CAN;
      Ctrl_ProcessFrame(frame.cmd, frame.seq, frame.payload, frame.len);
      break;
    }
  }
  Ctrl_CanResetReassembly();
}

static void Ctrl_CanProcessPacket(const Step42CanPacket_t *packet)
{
  uint8_t header;
  uint8_t payload_len;
  uint8_t required;

  if ((packet == 0) || (packet->len == 0U))
  {
    return;
  }
  if ((can_reassembly_len != 0U) &&
      ((uint32_t)(HAL_GetTick() - can_reassembly_last_ms) > STEP42_CAN_REASSEMBLY_TIMEOUT_MS))
  {
    Ctrl_CanResetReassembly();
  }

  header = packet->data[0];
  if (header == STEP42_CAN_SEGMENT_START)
  {
    if (packet->len < 3U)
    {
      Ctrl_CanResetReassembly();
      return;
    }
    payload_len = packet->data[1];
    if ((payload_len < 6U) || (payload_len > CTRL_MAX_FRAME_SIZE))
    {
      Ctrl_CanResetReassembly();
      return;
    }
    required = (payload_len > 6U) ? 6U : payload_len;
    if (packet->len != (uint8_t)(required + 2U))
    {
      Ctrl_CanResetReassembly();
      return;
    }
    can_reassembly_len = payload_len;
    can_reassembly_count = required;
    can_reassembly_next_index = 1U;
    can_reassembly_last_ms = HAL_GetTick();
    memcpy(can_reassembly, &packet->data[2], required);
    if (can_reassembly_count == can_reassembly_len)
    {
      Ctrl_CanProcessCompleteFrame();
    }
    return;
  }

  if ((header & 0xF0U) != STEP42_CAN_SEGMENT_CONTINUE)
  {
    return;
  }
  if ((can_reassembly_len == 0U) ||
      ((header & 0x0FU) != can_reassembly_next_index))
  {
    Ctrl_CanResetReassembly();
    return;
  }

  required = (uint8_t)(can_reassembly_len - can_reassembly_count);
  if (required > 7U)
  {
    required = 7U;
  }
  if (packet->len != (uint8_t)(required + 1U))
  {
    Ctrl_CanResetReassembly();
    return;
  }
  memcpy(&can_reassembly[can_reassembly_count], &packet->data[1], required);
  can_reassembly_count = (uint8_t)(can_reassembly_count + required);
  can_reassembly_next_index++;
  can_reassembly_last_ms = HAL_GetTick();
  if (can_reassembly_count == can_reassembly_len)
  {
    Ctrl_CanProcessCompleteFrame();
  }
}

static void Ctrl_CanProcessQueue(void)
{
  while (can_rx_tail != can_rx_head)
  {
    Step42CanPacket_t packet = can_rx_queue[can_rx_tail];
    can_rx_tail = (uint8_t)((can_rx_tail + 1U) & CAN_RX_QUEUE_MASK);
    Ctrl_CanProcessPacket(&packet);
  }
}

static void Ctrl_CanStop(void)
{
  if (can_started != 0U)
  {
    (void)HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    (void)HAL_FDCAN_Stop(&hfdcan1);
    can_started = 0U;
  }
  can_rx_head = can_rx_tail = 0U;
  Ctrl_CanResetReassembly();
}

static void Ctrl_CanStart(void)
{
  FDCAN_FilterTypeDef filter;

  if (can_started != 0U)
  {
    return;
  }

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  /* Accept node IDs 1..31 at 0x300..0x31F; exact node matching happens
     below so several Step42 boards can share a CAN bus. */
  filter.FilterID1 = STEP42_CAN_COMMAND_BASE;
  filter.FilterID2 = (uint32_t)(0x7FFU & ~STEP42_CAN_NODE_ID_MASK);
  if ((HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) ||
      (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
      (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) ||
      (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK))
  {
    (void)HAL_FDCAN_Stop(&hfdcan1);
    return;
  }

  can_started = 1U;
}

static void Ctrl_StartActiveReceiver(void)
{
  HAL_UART_AbortReceive(&huart2);
  HAL_UART_AbortReceive(&huart3);
  Ctrl_CanStop();

  if (Step42BoardMode_Get() == STEP42_BOARD_UART)
  {
    HAL_UART_Receive_IT(&huart2, &error_rx_byte, 1U);
    HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
  }
  else if (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG)
  {
    HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
  }
  else if (Step42BoardMode_Get() == STEP42_BOARD_CAN)
  {
    Ctrl_CanStart();
    /* CAN owns all motion commands. USART3 remains a restricted PC tuning
       channel for telemetry and tracking-PID parameters only. */
    HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
  }
}

void Step42Comms_Init(void)
{
  Step42Slave_ParserInit(&error_parser);
  Step42Slave_ParserInit(&debug_parser);
  Step42Slave_ParserInit(&can_parser);
  debug_stream_enable = 0U;
  vision_error_line_len = 0U;
  response_interface = CTRL_INTERFACE_ERROR_UART;
  stream_uart = &huart3;
  stream_interface = CTRL_INTERFACE_DEBUG_UART;
  error_rx_head = error_rx_tail = 0U;
  debug_rx_head = debug_rx_tail = 0U;
  can_rx_head = can_rx_tail = 0U;
  Ctrl_CanResetReassembly();
  Ctrl_StartActiveReceiver();
}

void Step42Comms_OnBoardModeChanged(void)
{
  Step42Slave_ParserInit(&error_parser);
  Step42Slave_ParserInit(&debug_parser);
  Step42Slave_ParserInit(&can_parser);
  debug_stream_enable = 0U;
  vision_error_line_len = 0U;
  response_interface = CTRL_INTERFACE_ERROR_UART;
  stream_uart = &huart3;
  stream_interface = CTRL_INTERFACE_DEBUG_UART;
  debug_tx_pause_until_ms = 0U;
  error_rx_head = error_rx_tail = 0U;
  debug_rx_head = debug_rx_tail = 0U;
  can_rx_head = can_rx_tail = 0U;
  Ctrl_CanResetReassembly();

  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_FLUSH_DRREGISTER(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_FLUSH_DRREGISTER(&huart3);
  Ctrl_StartActiveReceiver();
}

void Step42Comms_ProcessRx(void)
{
  if (Step42BoardMode_Get() == STEP42_BOARD_UART)
  {
    Ctrl_ProcessBuffer(error_rx_buffer,
                       &error_rx_head,
                       &error_rx_tail,
                       &error_parser,
                       &huart2,
                       0U);
    Ctrl_ProcessBuffer(debug_rx_buffer,
                       &debug_rx_head,
                       &debug_rx_tail,
                       &debug_parser,
                       &huart3,
                       1U);
  }
  else if (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG)
  {
    Ctrl_ProcessBuffer(debug_rx_buffer,
                       &debug_rx_head,
                       &debug_rx_tail,
                       &debug_parser,
                       &huart3,
                       0U);
  }
  else if (Step42BoardMode_Get() == STEP42_BOARD_CAN)
  {
    Ctrl_CanProcessQueue();
    Ctrl_ProcessBuffer(debug_rx_buffer,
                       &debug_rx_head,
                       &debug_rx_tail,
                       &debug_parser,
                       &huart3,
                       2U);
  }
}

void Step42Comms_SendStreamQueryIfDue(void)
{
  uint32_t now = HAL_GetTick();

  if ((debug_stream_enable != 0U) &&
      ((Step42BoardMode_Get() == STEP42_BOARD_UART) ||
       (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG) ||
       (Step42BoardMode_Get() == STEP42_BOARD_CAN)) &&
      (debug_tx_busy == 0U) &&
      ((uint32_t)(now - debug_raw_last_tx_ms) >= STATUS_STREAM_INTERVAL_MS) &&
      ((int32_t)(now - debug_tx_pause_until_ms) >= 0))
  {
    debug_raw_last_tx_ms = now;
    response_uart = stream_uart;
    response_interface = stream_interface;
    Step42Comms_SendTelemetry();
  }
}

void Step42Comms_OnUartError(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Step42Slave_ParserInit(&error_parser);
    error_rx_head = error_rx_tail = 0U;
    if (Step42BoardMode_Get() == STEP42_BOARD_UART)
    {
      HAL_UART_Receive_IT(&huart2, &error_rx_byte, 1U);
    }
  }
  else if (huart->Instance == USART3)
  {
    Step42Slave_ParserInit(&debug_parser);
    debug_rx_head = debug_rx_tail = 0U;
    if ((Step42BoardMode_Get() == STEP42_BOARD_UART) ||
        (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG) ||
        (Step42BoardMode_Get() == STEP42_BOARD_CAN))
    {
      HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
    }
  }
}

void Step42Comms_OnUartRxComplete(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Ctrl_RxPush(error_rx_buffer, &error_rx_head, &error_rx_tail, error_rx_byte);
    if (Step42BoardMode_Get() == STEP42_BOARD_UART)
    {
      HAL_UART_Receive_IT(&huart2, &error_rx_byte, 1U);
    }
  }
  else if (huart->Instance == USART3)
  {
    Ctrl_RxPush(debug_rx_buffer, &debug_rx_head, &debug_rx_tail, debug_rx_byte);
    if ((Step42BoardMode_Get() == STEP42_BOARD_UART) ||
        (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG) ||
        (Step42BoardMode_Get() == STEP42_BOARD_CAN))
    {
      HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
    }
  }
}

void Step42Comms_OnCanRxFifo0(void)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if ((Step42BoardMode_Get() != STEP42_BOARD_CAN) || (can_started == 0U))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      break;
    }
    if ((header.IdType == FDCAN_STANDARD_ID) &&
        (header.Identifier == (STEP42_CAN_COMMAND_BASE + Step42Control_GetCanNodeId())) &&
        (header.RxFrameType == FDCAN_DATA_FRAME) &&
        (header.DataLength > 0U) && (header.DataLength <= 8U))
    {
      Ctrl_CanPushPacket(data, (uint8_t)header.DataLength);
    }
  }
}
