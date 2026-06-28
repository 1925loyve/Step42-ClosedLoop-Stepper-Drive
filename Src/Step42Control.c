#include "Step42Control.h"
#include "Step42SlaveProtocol.h"
#include "Tmc2209.h"
#include "led.h"
#include "pid.h"

#define WEAK_PID_OUTPUT_SIGN (1.0f)
#define DEFAULT_CLOSED_ACCEL_STEPS (3000.0f)
#define WEAK_PID_ACCEL_STEPS DEFAULT_CLOSED_ACCEL_STEPS
#define MOTOR_BASE_STEPS_PER_REV (200.0f)
#define OPEN_POSITION_MAX_SPEED (2500.0f)
#define OPEN_POSITION_ACCEL (5000.0f)
#define STEP42_CFG_MAGIC (0x42323453UL)
#define STEP42_CFG_VERSION (1UL)
#define STEP42_CFG_ADDR (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define STEP42_CFG_FLASH_PAGE (FLASH_PAGE_NB - 1U)
#define STEP42_FLASH_WRITE_GUARD_MS (2000UL)

extern TIM_HandleTypeDef htim1;

Stepper_t Motor_42;

typedef struct
{
  uint16_t microsteps;
  float kp;
  float ki;
  float kd;
  float deadband;
  float accel;
} Step42DefaultParam_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t zero_valid;
  int32_t zero_cdeg;
  uint32_t home_mode;
  uint32_t crc;
} Step42FlashConfig_t;

static const Step42DefaultParam_t default_params[] = {
    {1U, 5.63f, 0.01f, 0.0f, 2.56f, 94.0f},
    {2U, 11.25f, 0.01f, 0.0f, 1.28f, 188.0f},
    {4U, 22.50f, 0.03f, 0.0f, 0.64f, 375.0f},
    {8U, 45.00f, 0.05f, 0.0f, 0.32f, 750.0f},
    {16U, 90.00f, 0.10f, 0.0f, 0.16f, 1500.0f},
    {32U, 180.00f, 0.20f, 0.0f, 0.08f, 3000.0f},
    {64U, 360.00f, 0.40f, 0.0f, 0.04f, 6000.0f},
    {128U, 720.00f, 0.80f, 0.0f, 0.02f, 12000.0f},
    {256U, 1440.00f, 1.60f, 0.0f, 0.01f, 24000.0f},
};

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
static uint16_t motor_microsteps = 32U;
static float motor_steps_per_rev = 6400.0f;
static float closed_position_accel = DEFAULT_CLOSED_ACCEL_STEPS;
static float zero_offset_deg = 0.0f;
static uint8_t zero_valid = 0U;
static uint8_t home_mode = CTRL_HOME_NEAREST;
static uint32_t last_flash_write_ms = 0U;

static float AbsF(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float Normalize360(float value)
{
  while (value >= 360.0f)
  {
    value -= 360.0f;
  }
  while (value < 0.0f)
  {
    value += 360.0f;
  }
  return value;
}

static uint32_t Config_CalcCrc(const Step42FlashConfig_t *cfg)
{
  const uint32_t *words = (const uint32_t *)cfg;
  uint32_t crc = 0xA5A55A5AUL;
  uint32_t count = (sizeof(Step42FlashConfig_t) / sizeof(uint32_t)) - 1U;
  uint32_t i;

  for (i = 0U; i < count; i++)
  {
    crc ^= words[i] + 0x9E3779B9UL + (crc << 6) + (crc >> 2);
  }

  return crc;
}

static void Config_Load(void)
{
  const Step42FlashConfig_t *cfg = (const Step42FlashConfig_t *)STEP42_CFG_ADDR;

  if ((cfg->magic != STEP42_CFG_MAGIC) ||
      (cfg->version != STEP42_CFG_VERSION) ||
      (cfg->crc != Config_CalcCrc(cfg)))
  {
    zero_valid = 0U;
    zero_offset_deg = 0.0f;
    home_mode = CTRL_HOME_NEAREST;
    return;
  }

  zero_valid = (cfg->zero_valid != 0UL) ? 1U : 0U;
  zero_offset_deg = Normalize360(((float)cfg->zero_cdeg) / 100.0f);
  if (cfg->home_mode <= CTRL_HOME_NEGATIVE)
  {
    home_mode = (uint8_t)cfg->home_mode;
  }
  else
  {
    home_mode = CTRL_HOME_NEAREST;
  }
}

static uint8_t Config_Save(void)
{
  union
  {
    Step42FlashConfig_t cfg;
    uint64_t words[sizeof(Step42FlashConfig_t) / sizeof(uint64_t)];
  } image;
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0U;
  uint32_t address = STEP42_CFG_ADDR;
  uint32_t i;

  if ((last_flash_write_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - last_flash_write_ms) < STEP42_FLASH_WRITE_GUARD_MS))
  {
    return CTRL_STATUS_BUSY;
  }

  image.cfg.magic = STEP42_CFG_MAGIC;
  image.cfg.version = STEP42_CFG_VERSION;
  image.cfg.zero_valid = zero_valid;
  image.cfg.zero_cdeg = (int32_t)(zero_offset_deg * 100.0f + 0.5f);
  image.cfg.home_mode = home_mode;
  image.cfg.crc = Config_CalcCrc(&image.cfg);

  Step42Control_StopMotion();
  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = STEP42_CFG_FLASH_PAGE;
  erase.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return CTRL_STATUS_RANGE;
  }

  for (i = 0U; i < (sizeof(Step42FlashConfig_t) / sizeof(uint64_t)); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, image.words[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return CTRL_STATUS_RANGE;
    }
    address += 8U;
  }

  HAL_FLASH_Lock();
  last_flash_write_ms = HAL_GetTick();
  return CTRL_STATUS_OK;
}

static const Step42DefaultParam_t *DefaultParams_Find(uint16_t microsteps)
{
  uint32_t i;

  for (i = 0U; i < (sizeof(default_params) / sizeof(default_params[0])); i++)
  {
    if (default_params[i].microsteps == microsteps)
    {
      return &default_params[i];
    }
  }

  return &default_params[5];
}

static void ApplyDefaultParams(uint16_t microsteps)
{
  const Step42DefaultParam_t *params = DefaultParams_Find(microsteps);

  closed_position_accel = params->accel;
  PID_Init(&weak_pid, params->kp, params->ki, params->kd, 0.001f, 5000.0f, 20.0f, params->deadband);
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

  Stepper_Set_Speed_Profile(&Motor_42, output, closed_position_accel);
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
}

static float MT6701_UpdateUnwrappedDeg(uint16_t raw_angle)
{
  int32_t delta_raw;
  float physical_angle;

  if (mt6701_unwrap_valid == 0U)
  {
    mt6701_last_raw = raw_angle;
    physical_angle = PID_RawToDeg(raw_angle);
    mt6701_unwrapped_deg = Normalize360(physical_angle - zero_offset_deg);
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
  steps = (int32_t)((target_unwrapped * motor_steps_per_rev / 360.0f) + ((target_unwrapped >= 0.0f) ? 0.5f : -0.5f));
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

  if ((motor_user_enabled == 0U) ||
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
  Config_Load();
  ApplyDefaultParams(motor_microsteps);
  Tmc2209_SetMicrosteps(motor_microsteps);
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
  if ((mode != CTRL_MODE_OPEN_SPEED) &&
      (mode != CTRL_MODE_OPEN_POSITION) &&
      (mode != CTRL_MODE_CLOSED_POSITION))
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
  if (control_mode == CTRL_MODE_CLOSED_POSITION)
  {
    Move_ToAbsoluteAngle(target_angle);
  }
}

uint8_t Step42Control_MoveStepsByMode(int32_t steps, float speed, float accel)
{
  float delta_deg;

  if (steps == 0)
  {
    return 0U;
  }

  if (control_mode == CTRL_MODE_OPEN_POSITION)
  {
    if (motor_user_enabled == 0U)
    {
      Step42Control_SetUserEnable(1U);
    }
    weak_pid_enable = 0U;
    closed_speed_target_steps = 0.0f;
    Motor_42.run_mode = MODE_POSITION;
    Stepper_Set_Speed_Profile(&Motor_42, speed, accel);
    Stepper_Move(&Motor_42, steps);
    led_white_on;
    return 1U;
  }

  if (control_mode == CTRL_MODE_CLOSED_POSITION)
  {
    if (motor_user_enabled == 0U)
    {
      Step42Control_SetUserEnable(1U);
    }
    delta_deg = ((float)steps) * 360.0f / motor_steps_per_rev;
    weak_pid_target_deg = mt6701_unwrapped_deg + delta_deg;
    PID_Reset(&weak_pid);
    weak_pid_enable = 1U;
    led_white_on;
    (void)speed;
    (void)accel;
    return 1U;
  }

  return 0U;
}

void Step42Control_RunSpeedByMode(float speed, float accel)
{
  if (control_mode != CTRL_MODE_OPEN_SPEED)
  {
    return;
  }

  Motor_42.run_mode = MODE_SPEED;
  OpenLoop_RunSpeed(speed, accel);
}

void Step42Control_StopMotion(void)
{
  weak_pid_enable = 0U;
  closed_speed_target_steps = 0.0f;
  Stepper_ForceStopPosition(&Motor_42);
  led_white_off;
}

uint8_t Step42Control_SetClosedAccel(float accel)
{
  if ((accel < 1.0f) || (accel > 65535.0f))
  {
    return 0U;
  }

  closed_position_accel = accel;
  return 1U;
}

uint8_t Step42Control_SetMicrosteps(uint16_t microsteps)
{
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

  Step42Control_StopMotion();
  if (Tmc2209_SetMicrosteps(microsteps) == 0U)
  {
    return 0U;
  }

  motor_microsteps = microsteps;
  motor_steps_per_rev = MOTOR_BASE_STEPS_PER_REV * (float)motor_microsteps;
  ApplyDefaultParams(microsteps);
  return 1U;
}

void Step42Control_SetPositionPID(float kp, float ki, float kd, float output_limit, float deadband)
{
  PID_Init(&weak_pid, kp, ki, kd, 0.001f, output_limit, 20.0f, deadband);
}

uint8_t Step42Control_SetHomeMode(uint8_t mode)
{
  uint8_t status;

  if (mode > CTRL_HOME_NEGATIVE)
  {
    return CTRL_STATUS_RANGE;
  }
  if (home_mode == mode)
  {
    return CTRL_STATUS_OK;
  }
  if ((last_flash_write_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - last_flash_write_ms) < STEP42_FLASH_WRITE_GUARD_MS))
  {
    return CTRL_STATUS_BUSY;
  }

  home_mode = mode;
  status = Config_Save();
  return status;
}

uint8_t Step42Control_SetCurrentZero(void)
{
  float new_offset = PID_RawToDeg(mt6701_raw_angle);

  if ((zero_valid != 0U) && (AbsF(PID_AngleErrorDeg(new_offset, zero_offset_deg)) < 0.01f))
  {
    return CTRL_STATUS_OK;
  }
  if ((last_flash_write_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - last_flash_write_ms) < STEP42_FLASH_WRITE_GUARD_MS))
  {
    return CTRL_STATUS_BUSY;
  }

  zero_offset_deg = new_offset;
  zero_valid = 1U;
  mt6701_angle_deg = 0.0f;
  mt6701_unwrapped_deg = 0.0f;
  mt6701_unwrap_valid = 0U;
  weak_pid_target_deg = 0.0f;
  PID_Reset(&weak_pid);
  return Config_Save();
}

uint8_t Step42Control_Home(uint8_t mode)
{
  float delta;

  if (mode > CTRL_HOME_NEGATIVE)
  {
    mode = home_mode;
  }
  if (control_mode != CTRL_MODE_CLOSED_POSITION)
  {
    return CTRL_STATUS_BAD_MODE;
  }

  if (mode == CTRL_HOME_NEAREST)
  {
    delta = PID_AngleErrorDeg(0.0f, mt6701_angle_deg);
  }
  else if (mode == CTRL_HOME_POSITIVE)
  {
    delta = 360.0f - mt6701_angle_deg;
    if (delta >= 360.0f)
    {
      delta = 0.0f;
    }
  }
  else
  {
    delta = -mt6701_angle_deg;
  }

  if (motor_user_enabled == 0U)
  {
    Step42Control_SetUserEnable(1U);
  }

  weak_pid_target_deg = mt6701_unwrapped_deg + delta;
  PID_Reset(&weak_pid);
  weak_pid_enable = 1U;
  led_white_on;
  return CTRL_STATUS_OK;
}

void Step42Control_GetPositionPID(float *kp, float *ki, float *kd, float *output_limit, float *deadband)
{
  if (kp != 0)
  {
    *kp = weak_pid.kp;
  }
  if (ki != 0)
  {
    *ki = weak_pid.ki;
  }
  if (kd != 0)
  {
    *kd = weak_pid.kd;
  }
  if (output_limit != 0)
  {
    *output_limit = weak_pid.output_limit;
  }
  if (deadband != 0)
  {
    *deadband = weak_pid.deadband;
  }
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
  mt6701_angle_deg = Normalize360(PID_RawToDeg(raw_angle) - zero_offset_deg);
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
      closed_speed_feedback_steps = (unwrapped_angle - speed_last_unwrapped_deg) * motor_steps_per_rev / 360.0f / 0.001f;
      speed_last_unwrapped_deg = unwrapped_angle;
    }
    WeakPID_Update(unwrapped_angle);
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

uint16_t Step42Control_GetMicrosteps(void)
{
  return motor_microsteps;
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

float Step42Control_GetClosedAccel(void)
{
  return closed_position_accel;
}

uint8_t Step42Control_GetHomeMode(void)
{
  return home_mode;
}

uint8_t Step42Control_IsZeroValid(void)
{
  return zero_valid;
}

MT6701_Status_t Step42Control_GetEncoderStatus(void)
{
  return mt6701_status;
}
