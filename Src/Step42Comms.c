#include "Step42Comms.h"
#include "Step42Control.h"
#include "Step42SlaveProtocol.h"

#define DEBUG_RAW_TX_INTERVAL_MS (20UL)

extern UART_HandleTypeDef huart2;

static Step42Slave_Parser_t ctrl_parser;
static volatile uint8_t debug_tx_busy = 0U;
static volatile uint32_t debug_tx_pause_until_ms = 0U;
static volatile uint32_t debug_raw_last_tx_ms = 0U;
static volatile uint32_t debug_rx_count = 0U;
static uint8_t debug_stream_enable = 1U;

static void Debug_SendBytes(uint8_t *buf, uint8_t len)
{
  debug_tx_busy = 1U;
  HAL_UART_Transmit(&huart2, buf, len, 30);
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

void Step42Comms_SendQuery(uint8_t seq)
{
  uint8_t resp[10];
  uint16_t angle_cdeg;
  int16_t pid_output;
  float angle_deg = Step42Control_GetAngleDeg();
  float pid_output_steps = Step42Control_GetPidOutputSteps();

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
  Ctrl_SendFrame(CTRL_CMD_QUERY, seq, resp, sizeof(resp));
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

    Step42Control_RunSpeedByMode((float)speed, (float)accel);
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  if (cmd == CTRL_CMD_PID)
  {
    int16_t kp_x100;
    int16_t ki_x100;
    int16_t kd_x100;
    uint16_t limit;
    uint16_t dead_x1000;

    if (len != 10U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_BAD_LEN);
      return;
    }

    kp_x100 = Step42Slave_ReadI16(payload);
    ki_x100 = Step42Slave_ReadI16(payload + 2U);
    kd_x100 = Step42Slave_ReadI16(payload + 4U);
    limit = Step42Slave_ReadU16(payload + 6U);
    dead_x1000 = Step42Slave_ReadU16(payload + 8U);
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
    if (payload[0] > CTRL_MODE_CLOSED_SPEED)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    Step42Control_SetMode(payload[0]);
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

  Ctrl_SendStatus(cmd, seq, CTRL_STATUS_UNKNOWN_CMD);
}

static void Ctrl_ProcessRxByte(uint8_t ch)
{
  Step42Slave_Frame_t frame;

  if (Step42Slave_ProcessByte(&ctrl_parser, ch, &frame) != 0U)
  {
    Ctrl_ProcessFrame(frame.cmd, frame.seq, frame.payload, frame.len);
  }
}

void Step42Comms_Init(void)
{
  Step42Slave_ParserInit(&ctrl_parser);
}

void Step42Comms_ProcessRx(void)
{
  while ((huart2.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    debug_rx_count++;
    Ctrl_ProcessRxByte((uint8_t)(huart2.Instance->RDR & 0xFFU));
  }

  if ((huart2.Instance->ISR & USART_ISR_ORE) != 0U)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
  }
}

void Step42Comms_SendStreamQueryIfDue(void)
{
  uint32_t now = HAL_GetTick();

  if ((debug_stream_enable != 0U) &&
      (debug_tx_busy == 0U) &&
      ((uint32_t)(now - debug_raw_last_tx_ms) >= DEBUG_RAW_TX_INTERVAL_MS) &&
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
    Step42Slave_ParserInit(&ctrl_parser);
  }
}
