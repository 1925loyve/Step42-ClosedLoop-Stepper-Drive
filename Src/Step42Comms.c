#include "Step42Comms.h"
#include "Step42Control.h"
#include "Step42SlaveProtocol.h"
#include "Step42BoardMode.h"
#include "Step42Sensors.h"
#include "Step42Tracking.h"
#include "Tmc2209.h"

#define STATUS_STREAM_INTERVAL_MS (100UL)
#define CTRL_RX_BUFFER_SIZE (128U)
#define CTRL_RX_BUFFER_MASK (CTRL_RX_BUFFER_SIZE - 1U)

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static Step42Slave_Parser_t error_parser;
static Step42Slave_Parser_t debug_parser;
static UART_HandleTypeDef *active_uart = &huart2;
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

static void Debug_SendBytes(uint8_t *buf, uint8_t len)
{
  debug_tx_busy = 1U;
  HAL_UART_Transmit(active_uart, buf, len, 30);
  debug_tx_pause_until_ms = HAL_GetTick() + 50U;
  debug_tx_busy = 0U;
}

static void Ctrl_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  uint8_t frame[CTRL_MAX_FRAME_SIZE];
  uint8_t frame_len;

  frame_len = Step42Slave_BuildResponse(frame, sizeof(frame), cmd, seq, payload, len);
  if (frame_len > 0U)
  {
    Debug_SendBytes(frame, frame_len);
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

void Step42Comms_SendQuery(uint8_t seq)
{
  uint8_t resp[54];
  uint16_t angle_cdeg;
  uint16_t target_cdeg;
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
  target_cdeg = (uint16_t)(Step42Control_GetPositionTargetDeg() * 100.0f + 0.5f);
  if (target_cdeg >= 36000U)
  {
    target_cdeg = 0U;
  }
  Step42Slave_WriteU16(resp, 49U, target_cdeg);
  Step42Slave_WriteI16(resp, 51U,
                       Ctrl_ClampI16(Step42Control_GetPositionErrorDeg() * 100.0f));
  resp[53] = Step42Control_IsPositionPidActive();
  Ctrl_SendFrame(CTRL_CMD_QUERY, seq, resp, sizeof(resp));
}

static void Step42Comms_SendTrackingQuery(uint8_t seq)
{
  uint8_t resp[59];
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
  Ctrl_SendFrame(CTRL_CMD_TRACK_QUERY, seq, resp, sizeof(resp));
}

static void Ctrl_ProcessFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
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
    uint16_t target_cdeg;
    float target_angle;

    if (len != 2U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    target_cdeg = Step42Slave_ReadU16(payload);
    if (target_cdeg > 36000U)
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

    debug_stream_enable = (payload[0] != 0U) ? 1U : 0U;
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
        (payload[0] != CTRL_MODE_TRACK_PID))
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

    Step42Control_StopMotion();
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
    if ((mode != CTRL_MODE_OPEN_POSITION) && (mode != CTRL_MODE_CLOSED_POSITION))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }

    if (Step42Control_MoveStepsByMode(steps, (float)speed, (float)accel) == 0U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

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
    if (len != 4U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }
    if (Step42Control_GetMode() != CTRL_MODE_TRACK_PID)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_MODE);
      return;
    }
    Step42Control_SetTrackingInput(((float)Step42Slave_ReadI16(payload)) / 100.0f,
                                   ((float)Step42Slave_ReadI16(payload + 2U)) / 100.0f);
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

  Ctrl_SendStatus(cmd, seq, CTRL_STATUS_UNKNOWN_CMD);
}

static uint8_t Ctrl_DebugCommandAllowed(uint8_t cmd)
{
  switch (cmd)
  {
  case CTRL_CMD_PING:
  case CTRL_CMD_ENABLE:
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
    return 1U;
  default:
    return 0U;
  }
}

static void Ctrl_ProcessRxByte(Step42Slave_Parser_t *parser, uint8_t ch, uint8_t restricted)
{
  Step42Slave_Frame_t frame;

  if (Step42Slave_ProcessByte(parser, ch, &frame) != 0U)
  {
    if ((restricted != 0U) && (Ctrl_DebugCommandAllowed(frame.cmd) == 0U))
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
                               uint8_t restricted)
{
  while (*tail != *head)
  {
    uint8_t value = buffer[*tail];
    *tail = (uint16_t)((*tail + 1U) & CTRL_RX_BUFFER_MASK);
    debug_rx_count++;
    Ctrl_ProcessRxByte(parser, value, restricted);
  }
}

static void Ctrl_StartActiveReceiver(void)
{
  HAL_UART_AbortReceive(&huart2);
  HAL_UART_AbortReceive(&huart3);

  if (Step42BoardMode_Get() == STEP42_BOARD_UART)
  {
    active_uart = &huart2;
    HAL_UART_Receive_IT(&huart2, &error_rx_byte, 1U);
  }
  else if (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG)
  {
    active_uart = &huart3;
    HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
  }
}

void Step42Comms_Init(void)
{
  Step42Slave_ParserInit(&error_parser);
  Step42Slave_ParserInit(&debug_parser);
  debug_stream_enable = 0U;
  error_rx_head = error_rx_tail = 0U;
  debug_rx_head = debug_rx_tail = 0U;
  Ctrl_StartActiveReceiver();
}

void Step42Comms_OnBoardModeChanged(void)
{
  Step42Slave_ParserInit(&error_parser);
  Step42Slave_ParserInit(&debug_parser);
  debug_stream_enable = 0U;
  debug_tx_pause_until_ms = 0U;
  error_rx_head = error_rx_tail = 0U;
  debug_rx_head = debug_rx_tail = 0U;

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
                       0U);
  }
  else if (Step42BoardMode_Get() == STEP42_BOARD_FACTORY_DEBUG)
  {
    Ctrl_ProcessBuffer(debug_rx_buffer,
                       &debug_rx_head,
                       &debug_rx_tail,
                       &debug_parser,
                       1U);
  }
}

void Step42Comms_SendStreamQueryIfDue(void)
{
  uint32_t now = HAL_GetTick();

  if ((debug_stream_enable != 0U) &&
      (debug_tx_busy == 0U) &&
      ((uint32_t)(now - debug_raw_last_tx_ms) >= STATUS_STREAM_INTERVAL_MS) &&
      ((int32_t)(now - debug_tx_pause_until_ms) >= 0))
  {
    debug_raw_last_tx_ms = now;
    Step42Comms_SendQuery(0U);
  }
}

void Step42Comms_OnUartError(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Step42Slave_ParserInit(&error_parser);
    error_rx_head = error_rx_tail = 0U;
    HAL_UART_Receive_IT(&huart2, &error_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    Step42Slave_ParserInit(&debug_parser);
    debug_rx_head = debug_rx_tail = 0U;
    HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
  }
}

void Step42Comms_OnUartRxComplete(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Ctrl_RxPush(error_rx_buffer, &error_rx_head, &error_rx_tail, error_rx_byte);
    HAL_UART_Receive_IT(&huart2, &error_rx_byte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    Ctrl_RxPush(debug_rx_buffer, &debug_rx_head, &debug_rx_tail, debug_rx_byte);
    HAL_UART_Receive_IT(&huart3, &debug_rx_byte, 1U);
  }
}
