#include "Step42Control.h"
#include "Step42SlaveProtocol.h"
#include "Tmc2209.h"
#include "led.h"
#include "pid.h"

#define WEAK_PID_OUTPUT_SIGN (1.0f)
#define WEAK_PID_ACCEL_STEPS (3000.0f)
#define MOTOR_STEPS_PER_REV (6400.0f)
#define OPEN_POSITION_MAX_SPEED (2500.0f)
#define OPEN_POSITION_ACCEL (5000.0f)

extern TIM_HandleTypeDef htim1;

Stepper_t Motor_42;

static volatile uint16_t mt6701_raw_angle = 0;
static volatile MT6701_Status_t mt6701_status = MT6701_HAL_ERROR;
static volatile float mt6701_angle_deg = 0.0f;
static volatile float mt6701_unwrapped_deg = 0.0f;
static volatile float weak_pid_target_deg = 0.0f;
static volatile float weak_pid_output_steps = 0.0f;
static volatile uint8_t weak_pid_enable = 0U;

static PID_t weak_pid;
static PID_t speed_pid;
static uint16_t mt6701_last_raw = 0U;
static uint8_t mt6701_unwrap_valid = 0U;
static uint8_t motor_user_enabled = 0U;
static uint8_t control_mode = CTRL_MODE_CLOSED_POSITION;
static float closed_speed_target_steps = 0.0f;
static float closed_speed_feedback_steps = 0.0f;
static uint8_t speed_feedback_valid = 0U;
static float speed_last_unwrapped_deg = 0.0f;

static float AbsF(float value)
{
  return (value >= 0.0f) ? value : -value;
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

static float MapAngleToCurrentCycle(float target_angle)
{
  float cycle_start;

  if (target_angle >= 360.0f)
  {
    target_angle = 360.0f;
  }
  else if (target_angle < 0.0f)
  {
    target_angle = 0.0f;
  }

  cycle_start = mt6701_unwrapped_deg - mt6701_angle_deg;
  return cycle_start + target_angle;
}

static void Move_ToAbsoluteAngle(float target_angle)
{
  if (motor_user_enabled == 0U)
  {
    Step42Control_SetUserEnable(1U);
  }

  weak_pid_target_deg = MapAngleToCurrentCycle(target_angle);
  PID_Reset(&weak_pid);
  weak_pid_enable = 1U;
  led_white_on;
}

static void Move_ToOpenLoopAngle(float target_angle)
{
  float target_unwrapped;
  int32_t steps;

  target_unwrapped = MapAngleToCurrentCycle(target_angle);
  target_unwrapped -= mt6701_unwrapped_deg;
  steps = (int32_t)((target_unwrapped * MOTOR_STEPS_PER_REV / 360.0f) + ((target_unwrapped >= 0.0f) ? 0.5f : -0.5f));
  if (steps == 0)
  {
    return;
  }

  if (motor_user_enabled == 0U)
  {
    Step42Control_SetUserEnable(1U);
  }

  weak_pid_enable = 0U;
  closed_speed_target_steps = 0.0f;
  Motor_42.run_mode = MODE_POSITION;
  Stepper_Set_Speed_Profile(&Motor_42, OPEN_POSITION_MAX_SPEED, OPEN_POSITION_ACCEL);
  Stepper_Move(&Motor_42, steps);
  led_white_on;
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
    Step42Control_StopMotion();
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

void Step42Control_Init(void)
{
  Stepper_Init(&Motor_42, &htim1, TIM_CHANNEL_1, tmc_DIR_GPIO_Port, tmc_DIR_Pin, MODE_SPEED);
  PID_Init(&weak_pid, 180.0f, 0.2f, 0.0f, 0.001f, 5000.0f, 20.0f, 0.08f);
  PID_Init(&speed_pid, 0.25f, 1.0f, 0.0f, 0.001f, 1500.0f, 1000.0f, 5.0f);
  Stepper_EN(tmc_EN_GPIO_Port, tmc_EN_Pin, TMC_UNEN);
}

void Step42Control_SetUserEnable(uint8_t enable)
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

void Step42Control_SetMode(uint8_t mode)
{
  if (mode > CTRL_MODE_CLOSED_SPEED)
  {
    return;
  }

  Step42Control_StopMotion();
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

void Step42Control_MoveToAngleByMode(float target_angle)
{
  if (control_mode == CTRL_MODE_OPEN_POSITION)
  {
    Move_ToOpenLoopAngle(target_angle);
  }
  else
  {
    if (control_mode != CTRL_MODE_CLOSED_POSITION)
    {
      Step42Control_SetMode(CTRL_MODE_CLOSED_POSITION);
    }
    Move_ToAbsoluteAngle(target_angle);
  }
}

void Step42Control_RunSpeedByMode(float speed, float accel)
{
  if (control_mode == CTRL_MODE_CLOSED_SPEED)
  {
    ClosedSpeed_Run(speed, accel);
  }
  else
  {
    if (control_mode != CTRL_MODE_OPEN_SPEED)
    {
      Step42Control_SetMode(CTRL_MODE_OPEN_SPEED);
    }
    Motor_42.run_mode = MODE_SPEED;
    OpenLoop_RunSpeed(speed, accel);
  }
}

void Step42Control_StopMotion(void)
{
  weak_pid_enable = 0U;
  closed_speed_target_steps = 0.0f;
  Stepper_ForceStopPosition(&Motor_42);
  led_white_off;
}

void Step42Control_SetPositionPID(float kp, float ki, float kd, float output_limit, float deadband)
{
  PID_Init(&weak_pid, kp, ki, kd, 0.001f, output_limit, 20.0f, deadband);
}

void Step42Control_HandleKey(int8_t key_flag)
{
  switch (key_flag)
  {
  case 1:
    if (motor_user_enabled != 0U)
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
      Step42Control_SetUserEnable(1U);
    }
    else
    {
      Step42Control_SetUserEnable(0U);
    }
    break;

  default:
    break;
  }
}

uint8_t Step42Control_TimerTick1kHz(void)
{
  uint16_t raw_angle = 0U;

  mt6701_status = MT6701_ReadRawAngleNoCrc(&raw_angle);
  if (mt6701_status != MT6701_OK)
  {
    return 0U;
  }

  mt6701_raw_angle = raw_angle;
  mt6701_angle_deg = PID_RawToDeg(raw_angle);
  {
    float unwrapped_angle = MT6701_UpdateUnwrappedDeg(raw_angle);
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
  }

  return 1U;
}

uint8_t Step42Control_IsUserEnabled(void)
{
  return motor_user_enabled;
}

uint8_t Step42Control_GetMode(void)
{
  return control_mode;
}

uint16_t Step42Control_GetRawAngle(void)
{
  return mt6701_raw_angle;
}

float Step42Control_GetAngleDeg(void)
{
  return mt6701_angle_deg;
}

float Step42Control_GetPidOutputSteps(void)
{
  return weak_pid_output_steps;
}

MT6701_Status_t Step42Control_GetEncoderStatus(void)
{
  return mt6701_status;
}
