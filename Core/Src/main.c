/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Src/Stepper.h"
#include "../../Src/tmc2209.h"
#include "../../Src/delay.h"
#include "../../Src/led.h"
#include "../../Src/MT6701.h"
#include "../../Src/Key.h"
#include "../../Src/pid.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Stepper_t Motor_42;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WEAK_PID_OUTPUT_SIGN (1.0f)
#define WEAK_PID_ACCEL_STEPS (3000.0f)
#define MOTOR_STEPS_PER_REV (6400.0f)
#define OPEN_POSITION_MAX_SPEED (2500.0f)
#define OPEN_POSITION_ACCEL (5000.0f)
#define DEBUG_RX_BUF_SIZE (64U)
#define DEBUG_RAW_TX_INTERVAL_MS (20UL)
#define CTRL_SOF1 (0xA5U)
#define CTRL_SOF2 (0x5AU)
#define CTRL_MAX_PAYLOAD (24U)
#define CTRL_PROTO_VERSION (1U)
#define CTRL_CMD_PING (0x01U)
#define CTRL_CMD_ENABLE (0x02U)
#define CTRL_CMD_ANGLE (0x03U)
#define CTRL_CMD_SPEED (0x04U)
#define CTRL_CMD_PID (0x05U)
#define CTRL_CMD_STREAM (0x06U)
#define CTRL_CMD_QUERY (0x07U)
#define CTRL_CMD_MODE (0x08U)
#define CTRL_CMD_STOP (0x09U)
#define CTRL_STATUS_OK (0x00U)
#define CTRL_STATUS_BAD_LEN (0x01U)
#define CTRL_STATUS_RANGE (0x02U)
#define CTRL_STATUS_UNKNOWN_CMD (0x03U)

#define CTRL_MODE_OPEN_POSITION (0x00U)
#define CTRL_MODE_CLOSED_POSITION (0x01U)
#define CTRL_MODE_OPEN_SPEED (0x02U)
#define CTRL_MODE_CLOSED_SPEED (0x03U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t mt6701_raw_angle = 0;
volatile MT6701_Status_t mt6701_status = MT6701_HAL_ERROR;
volatile float mt6701_angle_deg = 0.0f;
volatile float mt6701_unwrapped_deg = 0.0f;
volatile float weak_pid_target_deg = 0.0f;
volatile float weak_pid_output_steps = 0.0f;
volatile uint8_t weak_pid_enable = 0;
PID_t weak_pid;
PID_t speed_pid;
static uint16_t mt6701_last_raw = 0;
static uint8_t mt6701_unwrap_valid = 0;
static uint8_t debug_rx_buf[DEBUG_RX_BUF_SIZE];
static uint8_t debug_line_buf[DEBUG_RX_BUF_SIZE];
static uint8_t debug_rx_len = 0;
static volatile uint8_t debug_line_ready = 0;
static volatile uint8_t debug_tx_busy = 0;
static volatile uint32_t debug_tx_pause_until_ms = 0;
static volatile uint32_t debug_raw_last_tx_ms = 0;
static volatile uint32_t debug_rx_count = 0;
static uint32_t debug_rx_count_last_tx_ms = 0;
static uint8_t motor_user_enabled = 0;
static uint8_t debug_stream_enable = 1U;
static uint8_t ctrl_rx_state = 0;
static uint8_t ctrl_rx_len = 0;
static uint8_t ctrl_rx_cmd = 0;
static uint8_t ctrl_rx_seq = 0;
static uint8_t ctrl_rx_index = 0;
static uint8_t ctrl_rx_payload[CTRL_MAX_PAYLOAD];
static uint8_t control_mode = CTRL_MODE_CLOSED_POSITION;
static float closed_speed_target_steps = 0.0f;
static float closed_speed_feedback_steps = 0.0f;
static uint8_t speed_feedback_valid = 0U;
static float speed_last_unwrapped_deg = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_EnableOutput(void);
static void Motor_DisableOutput(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t MT6701_FormatRawLine(uint16_t raw_angle, uint8_t *buf)
{
  uint8_t tmp[5];
  uint8_t len = 0;
  uint8_t i = 0;

  if (raw_angle == 0U)
  {
    buf[len++] = '0';
  }
  else
  {
    while ((raw_angle > 0U) && (i < sizeof(tmp)))
    {
      tmp[i++] = (uint8_t)('0' + (raw_angle % 10U));
      raw_angle /= 10U;
    }

    while (i > 0U)
    {
      buf[len++] = tmp[--i];
    }
  }

  buf[len++] = '\r';
  buf[len++] = '\n';
  return len;
}

static void Stepper_ForceStopPosition(Stepper_t *stepper)
{
  if ((stepper == 0) || (stepper->htim == 0))
  {
    return;
  }

  stepper->state = STOP;
  stepper->current_speed = 0.0f;
  __HAL_TIM_DISABLE_IT(stepper->htim, TIM_IT_UPDATE);
  HAL_TIM_PWM_Stop(stepper->htim, stepper->tim_channel);
  __HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, 0);
}

static float AbsF(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static void WeakPID_StopMotor(void)
{
  Stepper_Stop(&Motor_42);
  weak_pid_output_steps = 0.0f;
}

static void WeakPID_Update(float current_angle)
{
  float output;

  if (weak_pid_enable == 0U)
  {
    return;
  }

  output = WEAK_PID_OUTPUT_SIGN * PID_UpdateLinear(&weak_pid, weak_pid_target_deg, current_angle);
  weak_pid_output_steps = output;

  if (AbsF(output) < 1.0f)
  {
    WeakPID_StopMotor();
    return;
  }

  if ((Motor_42.state != STOP) &&
      (((Motor_42.current_speed > 0.0f) && (output < 0.0f)) ||
       ((Motor_42.current_speed < 0.0f) && (output > 0.0f))))
  {
    Stepper_Stop(&Motor_42);
  }

  Stepper_Set_Speed_Profile(&Motor_42, output, WEAK_PID_ACCEL_STEPS);
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
}

static float MT6701_UpdateUnwrappedDeg(uint16_t raw_angle)
{
  int32_t delta_raw;

  if (mt6701_unwrap_valid == 0U)
  {
    mt6701_last_raw = raw_angle;
    mt6701_unwrapped_deg = PID_RawToDeg(raw_angle);
    mt6701_unwrap_valid = 1U;
    return mt6701_unwrapped_deg;
  }

  delta_raw = (int32_t)raw_angle - (int32_t)mt6701_last_raw;
  if (delta_raw > 8192)
  {
    delta_raw -= 16384;
  }
  else if (delta_raw < -8192)
  {
    delta_raw += 16384;
  }

  mt6701_last_raw = raw_angle;
  mt6701_unwrapped_deg += ((float)delta_raw) * 360.0f / 16384.0f;
  return mt6701_unwrapped_deg;
}

static uint8_t Debug_AppendChar(char *buf, uint8_t len, char ch)
{
  buf[len++] = ch;
  return len;
}

static uint8_t Debug_AppendText(char *buf, uint8_t len, const char *text)
{
  while (*text != '\0')
  {
    buf[len++] = *text++;
  }
  return len;
}

static uint8_t Debug_AppendUint(char *buf, uint8_t len, uint32_t value)
{
  char tmp[10];
  uint8_t i = 0;

  if (value == 0U)
  {
    buf[len++] = '0';
    return len;
  }

  while ((value > 0U) && (i < sizeof(tmp)))
  {
    tmp[i++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (i > 0U)
  {
    buf[len++] = tmp[--i];
  }
  return len;
}

static uint8_t Debug_AppendHexByte(char *buf, uint8_t len, uint8_t value)
{
  static const char hex[] = "0123456789ABCDEF";

  buf[len++] = hex[(value >> 4) & 0x0FU];
  buf[len++] = hex[value & 0x0FU];
  return len;
}

static uint8_t Ctrl_Crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0x00U;

  while (len-- > 0U)
  {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8U; bit++)
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

static int16_t Ctrl_ReadI16(const uint8_t *payload)
{
  return (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
}

static uint16_t Ctrl_ReadU16(const uint8_t *payload)
{
  return (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
}

static void Ctrl_WriteU16(uint8_t *payload, uint8_t index, uint16_t value)
{
  payload[index] = (uint8_t)(value & 0xFFU);
  payload[index + 1U] = (uint8_t)(value >> 8);
}

static void Ctrl_WriteI16(uint8_t *payload, uint8_t index, int16_t value)
{
  Ctrl_WriteU16(payload, index, (uint16_t)value);
}

static uint8_t Debug_AppendFloat3(char *buf, uint8_t len, float value)
{
  uint32_t scaled;

  if (value < 0.0f)
  {
    buf[len++] = '-';
    value = -value;
  }

  scaled = (uint32_t)(value * 1000.0f + 0.5f);
  len = Debug_AppendUint(buf, len, scaled / 1000U);
  buf[len++] = '.';
  buf[len++] = (char)('0' + ((scaled / 100U) % 10U));
  buf[len++] = (char)('0' + ((scaled / 10U) % 10U));
  buf[len++] = (char)('0' + (scaled % 10U));
  return len;
}

static void Debug_SendBytes(uint8_t *buf, uint8_t len)
{
  debug_tx_busy = 1U;
  HAL_UART_Transmit(&huart2, buf, len, 30);
  debug_tx_pause_until_ms = HAL_GetTick() + 50U;
  debug_tx_busy = 0U;
}

static void Ctrl_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  uint8_t frame[CTRL_MAX_PAYLOAD + 6U];
  uint8_t crc_buf[CTRL_MAX_PAYLOAD + 3U];

  if (len > CTRL_MAX_PAYLOAD)
  {
    return;
  }

  frame[0] = CTRL_SOF2;
  frame[1] = CTRL_SOF1;
  frame[2] = len;
  frame[3] = (uint8_t)(cmd | 0x80U);
  frame[4] = seq;
  crc_buf[0] = len;
  crc_buf[1] = frame[3];
  crc_buf[2] = seq;

  for (uint8_t i = 0; i < len; i++)
  {
    frame[5U + i] = payload[i];
    crc_buf[3U + i] = payload[i];
  }

  frame[5U + len] = Ctrl_Crc8(crc_buf, (uint8_t)(len + 3U));
  Debug_SendBytes(frame, (uint8_t)(len + 6U));
}

static void Ctrl_SendStatus(uint8_t cmd, uint8_t seq, uint8_t status)
{
  uint8_t payload[1];

  payload[0] = status;
  Ctrl_SendFrame(cmd, seq, payload, sizeof(payload));
}

static void Debug_SendPID(void)
{
  char msg[80];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "PID ");
  len = Debug_AppendFloat3(msg, len, weak_pid.kp);
  len = Debug_AppendChar(msg, len, ' ');
  len = Debug_AppendFloat3(msg, len, weak_pid.ki);
  len = Debug_AppendChar(msg, len, ' ');
  len = Debug_AppendFloat3(msg, len, weak_pid.kd);
  len = Debug_AppendChar(msg, len, ' ');
  len = Debug_AppendFloat3(msg, len, weak_pid.output_limit);
  len = Debug_AppendChar(msg, len, ' ');
  len = Debug_AppendFloat3(msg, len, weak_pid.deadband);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_SendAngle(float angle, float move_delta)
{
  char msg[80];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "ANGLE ");
  len = Debug_AppendFloat3(msg, len, angle);
  len = Debug_AppendChar(msg, len, ' ');
  len = Debug_AppendFloat3(msg, len, move_delta);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_SendEnable(void)
{
  char msg[16];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "EN ");
  len = Debug_AppendUint(msg, len, motor_user_enabled);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_SendSpeed(float speed, float accel)
{
  char msg[64];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "SPEED ");
  len = Debug_AppendFloat3(msg, len, speed);
  len = Debug_AppendChar(msg, len, ' ');
  len = Debug_AppendFloat3(msg, len, accel);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_SendMT6701Debug(void)
{
  MT6701_Debug_t debug;
  MT6701_Status_t status;
  char msg[120];
  uint8_t len = 0;

  HAL_TIM_Base_Stop_IT(&htim6);
  status = MT6701_ReadDebug(&debug);
  HAL_TIM_Base_Start_IT(&htim6);

  len = Debug_AppendText(msg, len, "MTD ");
  len = Debug_AppendText(msg, len, "st=");
  len = Debug_AppendUint(msg, len, status);

  if (status == MT6701_OK)
  {
    len = Debug_AppendText(msg, len, " b=");
    len = Debug_AppendHexByte(msg, len, debug.rx[0]);
    len = Debug_AppendChar(msg, len, ' ');
    len = Debug_AppendHexByte(msg, len, debug.rx[1]);
    len = Debug_AppendChar(msg, len, ' ');
    len = Debug_AppendHexByte(msg, len, debug.rx[2]);
    len = Debug_AppendText(msg, len, " s10=");
    len = Debug_AppendUint(msg, len, debug.raw_shift10);
    len = Debug_AppendText(msg, len, " s9=");
    len = Debug_AppendUint(msg, len, debug.raw_shift9);
    len = Debug_AppendText(msg, len, " s8=");
    len = Debug_AppendUint(msg, len, debug.raw_shift8);
    len = Debug_AppendText(msg, len, " s7=");
    len = Debug_AppendUint(msg, len, debug.raw_shift7);
    len = Debug_AppendText(msg, len, " mag=");
    len = Debug_AppendUint(msg, len, debug.magnetic_status);
    len = Debug_AppendText(msg, len, " crc=");
    len = Debug_AppendUint(msg, len, debug.received_crc);
    len = Debug_AppendChar(msg, len, '/');
    len = Debug_AppendUint(msg, len, debug.calculated_crc);
    len = Debug_AppendText(msg, len, " ok=");
    len = Debug_AppendUint(msg, len, debug.crc_ok);
  }

  len = Debug_AppendText(msg, len, "\r\n");
  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_SendRxCount(void)
{
  char msg[24];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "RXCNT ");
  len = Debug_AppendUint(msg, len, debug_rx_count);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_SendStream(void)
{
  char msg[16];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "STREAM ");
  len = Debug_AppendUint(msg, len, debug_stream_enable);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void Debug_RxCountTask(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - debug_rx_count_last_tx_ms) >= 1000UL)
  {
    debug_rx_count_last_tx_ms = now;
    Debug_SendRxCount();
  }
}

static void Debug_SendError(const char *error)
{
  char msg[64];
  uint8_t len = 0;

  len = Debug_AppendText(msg, len, "ERR ");
  len = Debug_AppendText(msg, len, error);
  len = Debug_AppendText(msg, len, "\r\n");

  Debug_SendBytes((uint8_t *)msg, len);
}

static void OpenLoop_RunSpeed(float speed, float accel)
{
  weak_pid_enable = 0U;
  closed_speed_target_steps = 0.0f;

  if (motor_user_enabled == 0U)
  {
    Motor_EnableOutput();
    led_red_on;
    motor_user_enabled = 1U;
  }

  if (AbsF(speed) < 1.0f)
  {
    Stepper_Stop(&Motor_42);
    Motor_42.target_speed = 0.0f;
    Motor_42.current_speed = 0.0f;
    led_white_off;
    return;
  }

  Stepper_Set_Speed_Profile(&Motor_42, speed, accel);
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
  led_white_on;
}

static void Motor_SetUserEnable(uint8_t enable)
{
  if (enable != 0U)
  {
    Motor_EnableOutput();
    if (control_mode == CTRL_MODE_CLOSED_POSITION)
    {
      weak_pid_target_deg = mt6701_unwrapped_deg;
      PID_Reset(&weak_pid);
      weak_pid_enable = 1U;
    }
    else
    {
      weak_pid_enable = 0U;
    }
    led_red_on;
    motor_user_enabled = 1U;
  }
  else
  {
    Motor_DisableOutput();
    led_red_off;
    led_white_off;
    motor_user_enabled = 0U;
  }
}

static void Move_ToAbsoluteAngle(float target_angle)
{
  float move_delta;

  move_delta = PID_AngleErrorDeg(target_angle, mt6701_angle_deg);

  if (motor_user_enabled == 0U)
  {
    Motor_SetUserEnable(1U);
  }

  weak_pid_target_deg = mt6701_unwrapped_deg + move_delta;
  PID_Reset(&weak_pid);
  weak_pid_enable = 1U;
  led_white_on;
}

static void StopMotion(void)
{
  weak_pid_enable = 0U;
  closed_speed_target_steps = 0.0f;
  Stepper_ForceStopPosition(&Motor_42);
  led_white_off;
}

static void Control_SetMode(uint8_t mode)
{
  if (mode > CTRL_MODE_CLOSED_SPEED)
  {
    return;
  }

  StopMotion();
  control_mode = mode;
  if (mode == CTRL_MODE_OPEN_POSITION)
  {
    Motor_42.run_mode = MODE_POSITION;
  }
  else
  {
    Motor_42.run_mode = MODE_SPEED;
  }

  if ((motor_user_enabled != 0U) && (mode == CTRL_MODE_CLOSED_POSITION))
  {
    weak_pid_target_deg = mt6701_unwrapped_deg;
    PID_Reset(&weak_pid);
    weak_pid_enable = 1U;
  }
}

static void Move_ToOpenLoopAngle(float target_angle)
{
  float move_delta;
  int32_t steps;

  move_delta = PID_AngleErrorDeg(target_angle, mt6701_angle_deg);
  steps = (int32_t)((move_delta * MOTOR_STEPS_PER_REV / 360.0f) + ((move_delta >= 0.0f) ? 0.5f : -0.5f));
  if (steps == 0)
  {
    return;
  }

  if (motor_user_enabled == 0U)
  {
    Motor_SetUserEnable(1U);
  }

  weak_pid_enable = 0U;
  closed_speed_target_steps = 0.0f;
  Motor_42.run_mode = MODE_POSITION;
  Stepper_Set_Speed_Profile(&Motor_42, OPEN_POSITION_MAX_SPEED, OPEN_POSITION_ACCEL);
  Stepper_Move(&Motor_42, steps);
  led_white_on;
}

static void Move_ToAngleByMode(float target_angle)
{
  if (control_mode == CTRL_MODE_OPEN_POSITION)
  {
    Move_ToOpenLoopAngle(target_angle);
  }
  else
  {
    if (control_mode != CTRL_MODE_CLOSED_POSITION)
    {
      Control_SetMode(CTRL_MODE_CLOSED_POSITION);
    }
    Move_ToAbsoluteAngle(target_angle);
  }
}

static void ClosedSpeed_Run(float speed, float accel)
{
  (void)accel;

  if (motor_user_enabled == 0U)
  {
    Motor_EnableOutput();
    led_red_on;
    motor_user_enabled = 1U;
  }

  weak_pid_enable = 0U;
  Motor_42.run_mode = MODE_SPEED;
  closed_speed_target_steps = speed;
  PID_Reset(&speed_pid);

  if (AbsF(speed) < 1.0f)
  {
    StopMotion();
    return;
  }

  Stepper_Set_Speed_Profile(&Motor_42, speed, WEAK_PID_ACCEL_STEPS);
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
  led_white_on;
}

static void ClosedSpeed_Update(void)
{
  float correction;
  float command_speed;

  if ((control_mode != CTRL_MODE_CLOSED_SPEED) ||
      (motor_user_enabled == 0U) ||
      (AbsF(closed_speed_target_steps) < 1.0f) ||
      (speed_feedback_valid == 0U))
  {
    return;
  }

  correction = PID_UpdateLinear(&speed_pid, closed_speed_target_steps, closed_speed_feedback_steps);
  command_speed = closed_speed_target_steps + correction;
  if (command_speed > 5000.0f)
  {
    command_speed = 5000.0f;
  }
  else if (command_speed < -5000.0f)
  {
    command_speed = -5000.0f;
  }

  Stepper_Set_Speed_Profile(&Motor_42, command_speed, WEAK_PID_ACCEL_STEPS);
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
}

static void RunSpeedByMode(float speed, float accel)
{
  if (control_mode == CTRL_MODE_CLOSED_SPEED)
  {
    ClosedSpeed_Run(speed, accel);
  }
  else
  {
    if (control_mode != CTRL_MODE_OPEN_SPEED)
    {
      Control_SetMode(CTRL_MODE_OPEN_SPEED);
    }
    Motor_42.run_mode = MODE_SPEED;
    OpenLoop_RunSpeed(speed, accel);
  }
}

static void Ctrl_SendQuery(uint8_t seq)
{
  uint8_t resp[10];
  uint16_t angle_cdeg;
  int16_t pid_output;

  angle_cdeg = (uint16_t)(mt6701_angle_deg * 100.0f + 0.5f);
  if (angle_cdeg >= 36000U)
  {
    angle_cdeg = 0U;
  }
  if (weak_pid_output_steps > 32767.0f)
  {
    pid_output = 32767;
  }
  else if (weak_pid_output_steps < -32768.0f)
  {
    pid_output = -32768;
  }
  else
  {
    pid_output = (int16_t)weak_pid_output_steps;
  }

  resp[0] = CTRL_STATUS_OK;
  resp[1] = motor_user_enabled;
  resp[2] = debug_stream_enable;
  resp[3] = control_mode;
  Ctrl_WriteU16(resp, 4U, mt6701_raw_angle);
  Ctrl_WriteU16(resp, 6U, angle_cdeg);
  Ctrl_WriteI16(resp, 8U, pid_output);
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

    Motor_SetUserEnable((payload[0] != 0U) ? 1U : 0U);
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

    target_cdeg = Ctrl_ReadU16(payload);
    if (target_cdeg > 36000U)
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    target_angle = ((float)target_cdeg) / 100.0f;
    if (target_angle >= 360.0f)
    {
      target_angle = 0.0f;
    }
    Move_ToAngleByMode(target_angle);
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

    speed = Ctrl_ReadI16(payload);
    accel = Ctrl_ReadU16(payload + 2U);
    if ((speed < -5000) || (speed > 5000) || (accel == 0U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    RunSpeedByMode((float)speed, (float)accel);
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

    kp_x100 = Ctrl_ReadI16(payload);
    ki_x100 = Ctrl_ReadI16(payload + 2U);
    kd_x100 = Ctrl_ReadI16(payload + 4U);
    limit = Ctrl_ReadU16(payload + 6U);
    dead_x1000 = Ctrl_ReadU16(payload + 8U);
    if ((limit < 1U) || (dead_x1000 > 10000U))
    {
      Ctrl_SendStatus(cmd, seq, CTRL_STATUS_RANGE);
      return;
    }

    PID_Init(&weak_pid,
             ((float)kp_x100) / 100.0f,
             ((float)ki_x100) / 100.0f,
             ((float)kd_x100) / 100.0f,
             0.001f,
             (float)limit,
             20.0f,
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

    Ctrl_SendQuery(seq);
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

    Control_SetMode(payload[0]);
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

    StopMotion();
    Ctrl_SendStatus(cmd, seq, CTRL_STATUS_OK);
    return;
  }

  Ctrl_SendStatus(cmd, seq, CTRL_STATUS_UNKNOWN_CMD);
}

static uint8_t Ctrl_ProcessRxByte(uint8_t ch)
{
  uint8_t crc_buf[CTRL_MAX_PAYLOAD + 3U];
  uint8_t crc;

  switch (ctrl_rx_state)
  {
  case 0:
    if (ch == CTRL_SOF1)
    {
      ctrl_rx_state = 1U;
      return 1U;
    }
    return 0U;

  case 1:
    if (ch == CTRL_SOF2)
    {
      ctrl_rx_state = 2U;
      return 1U;
    }
    ctrl_rx_state = 0U;
    return 1U;

  case 2:
    ctrl_rx_len = ch;
    ctrl_rx_index = 0U;
    if (ctrl_rx_len > CTRL_MAX_PAYLOAD)
    {
      ctrl_rx_state = 0U;
      return 1U;
    }
    ctrl_rx_state = 3U;
    return 1U;

  case 3:
    ctrl_rx_cmd = ch;
    ctrl_rx_state = 4U;
    return 1U;

  case 4:
    ctrl_rx_seq = ch;
    ctrl_rx_state = (ctrl_rx_len == 0U) ? 6U : 5U;
    return 1U;

  case 5:
    ctrl_rx_payload[ctrl_rx_index++] = ch;
    if (ctrl_rx_index >= ctrl_rx_len)
    {
      ctrl_rx_state = 6U;
    }
    return 1U;

  case 6:
    crc_buf[0] = ctrl_rx_len;
    crc_buf[1] = ctrl_rx_cmd;
    crc_buf[2] = ctrl_rx_seq;
    for (uint8_t i = 0; i < ctrl_rx_len; i++)
    {
      crc_buf[3U + i] = ctrl_rx_payload[i];
    }
    crc = Ctrl_Crc8(crc_buf, (uint8_t)(ctrl_rx_len + 3U));
    if (crc == ch)
    {
      Ctrl_ProcessFrame(ctrl_rx_cmd, ctrl_rx_seq, ctrl_rx_payload, ctrl_rx_len);
    }
    ctrl_rx_state = 0U;
    return 1U;

  default:
    ctrl_rx_state = 0U;
    return 1U;
  }
}

static void Debug_ProcessLine(char *line)
{
  char *cursor;
  float kp;
  float ki;
  float kd;
  float output_limit;
  float deadband;

  if (strcmp(line, "PID?") == 0)
  {
    Debug_SendPID();
    return;
  }

  if (strcmp(line, "EN?") == 0)
  {
    Debug_SendEnable();
    return;
  }

  if (strcmp(line, "MTD?") == 0)
  {
    Debug_SendMT6701Debug();
    return;
  }

  if (strcmp(line, "STREAM?") == 0)
  {
    Debug_SendStream();
    return;
  }

  if (strncmp(line, "STREAM ", 7) == 0)
  {
    cursor = line + 7;
    if (*cursor == '1')
    {
      debug_stream_enable = 1U;
      Debug_SendStream();
      return;
    }
    if (*cursor == '0')
    {
      debug_stream_enable = 0U;
      Debug_SendStream();
      return;
    }

    Debug_SendError("STREAM_VALUE");
    return;
  }

  if (strncmp(line, "EN ", 3) == 0)
  {
    cursor = line + 3;
    if (*cursor == '1')
    {
      Motor_SetUserEnable(1U);
      Debug_SendEnable();
      return;
    }
    if (*cursor == '0')
    {
      Motor_SetUserEnable(0U);
      Debug_SendEnable();
      return;
    }

    Debug_SendError("EN_VALUE");
    return;
  }

  if (strncmp(line, "ANGLE ", 6) == 0)
  {
    float target_angle;

    cursor = line + 6;
    target_angle = strtof(cursor, &cursor);
    if ((target_angle < 0.0f) || (target_angle > 360.0f))
    {
      Debug_SendError("ANGLE_RANGE");
      return;
    }

    if (target_angle >= 360.0f)
    {
      target_angle = 0.0f;
    }

    Move_ToAbsoluteAngle(target_angle);
    return;
  }

  if (strncmp(line, "SPEED ", 6) == 0)
  {
    float speed;
    float accel;

    cursor = line + 6;
    speed = strtof(cursor, &cursor);
    accel = strtof(cursor, &cursor);

    if ((speed < -5000.0f) || (speed > 5000.0f))
    {
      Debug_SendError("SPEED_RANGE");
      return;
    }
    if (accel <= 0.0f)
    {
      Debug_SendError("ACCEL_RANGE");
      return;
    }

    OpenLoop_RunSpeed(speed, accel);
    return;
  }

  if (strncmp(line, "PID ", 4) != 0)
  {
    return;
  }

  cursor = line + 4;
  kp = strtof(cursor, &cursor);
  ki = strtof(cursor, &cursor);
  kd = strtof(cursor, &cursor);
  output_limit = strtof(cursor, &cursor);
  deadband = strtof(cursor, &cursor);

  if ((output_limit < 1.0f) || (deadband < 0.0f))
  {
    return;
  }

  PID_Init(&weak_pid, kp, ki, kd, 0.001f, output_limit, 20.0f, deadband);
  Debug_SendPID();
}

static void Debug_ProcessRxByte(uint8_t ch)
{
  debug_rx_count++;
  (void)Ctrl_ProcessRxByte(ch);
}

static void Debug_ProcessRx(void)
{
  while ((huart2.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    Debug_ProcessRxByte((uint8_t)(huart2.Instance->RDR & 0xFFU));
  }

  if ((huart2.Instance->ISR & USART_ISR_ORE) != 0U)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    debug_rx_len = 0U;
  }

  debug_line_ready = 0U;
}

static void Debug_RxStart(void)
{
}

static void Motor_EnableOutput(void)
{
  Stepper_EN(tmc_EN_GPIO_Port, tmc_EN_Pin, TMC_EN);
}

static void Motor_DisableOutput(void)
{
  weak_pid_enable = 0U;
  Stepper_ForceStopPosition(&Motor_42);
  Stepper_EN(tmc_EN_GPIO_Port, tmc_EN_Pin, TMC_UNEN);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  MT6701_Init();

  led_red_off;
  led_blue_off;
  led_white_off;

  uint8_t tmc_addr = 0xFF;
  if (Tmc2209_InitAuto(&tmc_addr))
  {
    led_blue_on;
  }
  else
  {
    led_red_on;
  }

  /* Keep motor and LED animation disabled while checking TMC2209 UART. */
  Debug_RxStart();
  HAL_TIM_Base_Start_IT(&htim6);

  Stepper_Init(&Motor_42, &htim1, TIM_CHANNEL_1, tmc_DIR_GPIO_Port, tmc_DIR_Pin, MODE_SPEED);
  PID_Init(&weak_pid, 180.0f, 0.2f, 0.0f, 0.001f, 5000.0f, 20.0f, 0.08f);
  PID_Init(&speed_pid, 0.25f, 1.0f, 0.0f, 0.001f, 1500.0f, 1000.0f, 5.0f);
  Stepper_EN(tmc_EN_GPIO_Port, tmc_EN_Pin, TMC_UNEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Debug_ProcessRx();

    int8_t key_flag = Key_Scan();
    switch (key_flag)
    {
    case 1:
      if (motor_user_enabled)
      {
        weak_pid_target_deg = mt6701_unwrapped_deg + 360.0f;
        PID_Reset(&weak_pid);
        weak_pid_enable = 1U;
        led_white_on;
      }
      break;
    case 2:
      Stepper_Stop(&Motor_42);
      if (motor_user_enabled == 0U)
      {
        Motor_SetUserEnable(1U);
      }
      else
      {
        Motor_SetUserEnable(0U);
      }
      break;
    default:
      break;
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 1;
  hfdcan1.Init.NominalTimeSeg2 = 1;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin | MT6701_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin | LED_WHITE_Pin | tmc_DIR_Pin, GPIO_PIN_RESET);

  /* TMC2209 ENN is active low. Keep the driver disabled during GPIO init. */
  HAL_GPIO_WritePin(tmc_EN_GPIO_Port, tmc_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_RED_Pin MT6701_CSN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin | MT6701_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_BLUE_Pin LED_WHITE_Pin tmc_DIR_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin | LED_WHITE_Pin | tmc_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : tmc_EN_Pin */
  GPIO_InitStruct.Pin = tmc_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(tmc_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin MS1_Pin MS2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin | MS1_Pin | MS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    uint16_t raw_angle = 0;

    mt6701_status = MT6701_ReadRawAngleNoCrc(&raw_angle);
    if (mt6701_status == MT6701_OK)
    {
      float unwrapped_angle;

      mt6701_raw_angle = raw_angle;
      mt6701_angle_deg = PID_RawToDeg(raw_angle);
      unwrapped_angle = MT6701_UpdateUnwrappedDeg(raw_angle);
      if (speed_feedback_valid == 0U)
      {
        speed_last_unwrapped_deg = unwrapped_angle;
        speed_feedback_valid = 1U;
        closed_speed_feedback_steps = 0.0f;
      }
      else
      {
        closed_speed_feedback_steps = (unwrapped_angle - speed_last_unwrapped_deg) * MOTOR_STEPS_PER_REV / 360.0f / 0.001f;
        speed_last_unwrapped_deg = unwrapped_angle;
      }
      WeakPID_Update(unwrapped_angle);
      ClosedSpeed_Update();
      if ((debug_stream_enable != 0U) &&
          (debug_tx_busy == 0U) &&
          ((uint32_t)(HAL_GetTick() - debug_raw_last_tx_ms) >= DEBUG_RAW_TX_INTERVAL_MS) &&
          ((int32_t)(HAL_GetTick() - debug_tx_pause_until_ms) >= 0))
      {
        debug_raw_last_tx_ms = HAL_GetTick();
        Ctrl_SendQuery(0U);
      }
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    debug_rx_len = 0U;
    Debug_RxStart();
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
