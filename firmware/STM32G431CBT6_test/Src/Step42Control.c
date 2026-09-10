#include "Step42Control.h"
#include "Step42SlaveProtocol.h"
#include "Tmc2209.h"
#include "led.h"
#include "Key.h"
#include "pid.h"
#include "Step42Tracking.h"
#include <string.h>

#define WEAK_PID_OUTPUT_SIGN (1.0f)
#define DEFAULT_CLOSED_ACCEL_STEPS (3000.0f)
#define WEAK_PID_ACCEL_STEPS DEFAULT_CLOSED_ACCEL_STEPS
#define MOTOR_BASE_STEPS_PER_REV (200.0f)
#define STEP42_BASE_PULSE_SPEED (5000U)
#define STEP42_HARD_MAX_PULSE_SPEED (30000U)
#define OPEN_POSITION_MAX_SPEED (2500.0f)
#define OPEN_POSITION_ACCEL (5000.0f)
#define STEP42_CFG_MAGIC (0x42323453UL)
#define STEP42_CFG_VERSION (4UL)
#define STEP42_CFG_ADDR (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define STEP42_CFG_FLASH_PAGE (FLASH_PAGE_NB - 1U)
#define STEP42_PROGRAM_MAGIC (0x50473432UL)
#define STEP42_PROGRAM_VERSION (2UL)
#define STEP42_PROGRAM_CFG_ADDR (FLASH_BASE + FLASH_SIZE - (2UL * FLASH_PAGE_SIZE))
#define STEP42_PROGRAM_FLASH_PAGE (FLASH_PAGE_NB - 2U)
#define STEP42_FLASH_WRITE_GUARD_MS (2000UL)
#define CONTROL_TIMER_TICK_HZ (1000000UL)
#define SPEED_ESTIMATE_WINDOW_S (0.020f)
#define SPEED_FILTER_ALPHA (0.25f)
#define WEAK_PID_MIN_CORRECTION_SPEED (80.0f)
#define ENCODER_UNWRAP_REBASE_DEG (7200.0f)
#define CLOSED_POSITION_LIMIT_DEG (7200.0f)
#define STEP42_CAN_NODE_ID_MIN (1U)
#define STEP42_CAN_NODE_ID_MAX (31U)
#define STALL_COMMAND_MIN_RPM (15.0f)
#define STALL_MIN_FEEDBACK_RPM (2.0f)
#define STALL_FEEDBACK_RATIO (0.12f)
#define STALL_POSITION_ERROR_DEG (1.0f)
#define STALL_STARTUP_GRACE_MS (300U)
#define STALL_CONFIRM_MS (500U)
#define STALL_REVERSE_MIN_RPM (3.0f)
#define STALL_REVERSE_CONFIRM_MS (250U)

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;

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
  float position_kp;
  float position_ki;
  float position_kd;
  float position_limit;
  float position_deadband;
  float closed_accel;
  Step42TrackingConfig_t tracking;
  uint32_t tracking_reverse;
  uint32_t reserved;
  Step42ExternalPositionConfig_t external_position;
  /* Keep CRC on an 8-byte Flash-programming boundary. */
  uint32_t alignment_padding;
  uint32_t crc;
} Step42FlashConfig_t;

/* Firmware v2 layout, retained so existing saved settings remain readable. */
typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t zero_valid;
  int32_t zero_cdeg;
  uint32_t home_mode;
  float position_kp;
  float position_ki;
  float position_kd;
  float position_limit;
  float position_deadband;
  float closed_accel;
  Step42TrackingConfig_t tracking;
  uint32_t tracking_reverse;
  uint32_t reserved;
  uint32_t crc;
} Step42FlashConfigV2_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t zero_valid;
  int32_t zero_cdeg;
  uint32_t home_mode;
  uint32_t crc;
} Step42FlashConfigV1_t;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t count;
  Step42ProgramStep_t steps[STEP42_PROGRAM_MAX_STEPS];
  uint32_t crc;
} Step42ProgramFlash_t;

typedef struct
{
  uint8_t valid;
  uint8_t mode;
  uint8_t user_enabled;
  uint8_t chopper_mode;
  uint16_t microsteps;
  uint8_t home_mode;
  float kp;
  float ki;
  float kd;
  float pid_limit;
  float deadband;
  float closed_accel;
} Step42ProgramSnapshot_t;

/* External continuous-position control owns its own microstep/PID set. */
typedef struct
{
  uint8_t valid;
  uint16_t microsteps;
  float kp;
  float ki;
  float kd;
  float pid_limit;
  float deadband;
  float closed_accel;
} Step42ExternalRuntimeSnapshot_t;

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
static uint32_t control_loop_hz = 1000UL;
static float speed_accum_delta_deg = 0.0f;
static float speed_accum_time_s = 0.0f;
static float motor_speed_rpm = 0.0f;
static uint8_t thermal_lockout = 0U;
static uint8_t stall_fault = 0U;
static uint16_t stall_motion_age_ms = 0U;
static uint16_t stall_elapsed_ms = 0U;
static uint16_t stall_reverse_elapsed_ms = 0U;
static int8_t stall_command_sign = 0;
static uint8_t external_task_marked = 0U;
static uint8_t tracking_error_reverse = 0U;
static uint8_t can_node_id = STEP42_CAN_NODE_ID_MIN;
static uint8_t program_mode = 0U;
static uint8_t program_running = 0U;
static uint8_t program_count = 0U;
static uint8_t program_ready = 0U;
static uint8_t program_editing = 0U;
static uint8_t program_save_armed = 0U;
static volatile uint8_t program_target_detected = 0U;
static volatile uint8_t program_custom_signal1 = 0U;
static volatile uint8_t program_custom_signal2 = 0U;
static Step42ExternalPositionConfig_t external_position_config;
static volatile float external_position_current_deg = 0.0f;
static volatile float external_position_target_deg = 0.0f;
static uint16_t external_position_last_raw = 0U;
static uint8_t external_position_unwrap_valid = 0U;
static uint32_t external_position_last_target_ms = 0U;
static Step42FlashConfig_t loaded_flash_config;
static uint8_t loaded_flash_config_valid = 0U;
static Step42ProgramSnapshot_t program_snapshot;
static Step42ExternalRuntimeSnapshot_t external_runtime_snapshot;
static uint8_t program_index = 0U;
static uint8_t program_step_started = 0U;
static uint32_t program_step_start_ms = 0U;
static uint32_t program_last_flash_write_ms = 0U;
static Step42ProgramStep_t program_steps[STEP42_PROGRAM_MAX_STEPS];

static uint32_t Program_CalcCrc(const Step42ProgramFlash_t *program);
static uint32_t ConfigV1_CalcCrc(const Step42FlashConfigV1_t *cfg);
static uint32_t ConfigV2_CalcCrc(const Step42FlashConfigV2_t *cfg);
static void Program_Load(void);
static void Program_CaptureRuntime(void);
static void Program_RestoreRuntime(void);
static uint8_t Program_ConditionMet(const Step42ProgramStep_t *step);
static void Program_Jump(uint8_t index);
static uint8_t Program_ApplyDriveState(const Step42ProgramStep_t *step);
static void ExternalPosition_ResetDefaults(void);
static void ExternalPosition_Rebase(void);
static float ExternalPosition_UpdateCurrent(uint16_t raw_angle);
static void ExternalPosition_ApplyConfig(void);
static void ExternalPosition_CaptureRuntime(void);
static void ExternalPosition_RestoreRuntime(void);
static void StallProtection_Update(void);

static float AbsF(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float ClampF(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

uint16_t Step42Control_GetPulseSpeedLimitForMicrosteps(uint16_t microsteps)
{
  uint32_t limit;

  switch (microsteps)
  {
  case 1U:
  case 2U:
  case 4U:
  case 8U:
  case 16U:
  case 32U:
  case 64U:
  case 128U:
  case 256U:
    break;
  default:
    return STEP42_BASE_PULSE_SPEED;
  }

  limit = (uint32_t)STEP42_BASE_PULSE_SPEED * (uint32_t)microsteps;
  if (limit > STEP42_HARD_MAX_PULSE_SPEED)
  {
    limit = STEP42_HARD_MAX_PULSE_SPEED;
  }
  return (uint16_t)limit;
}

uint16_t Step42Control_GetPulseSpeedLimit(void)
{
  return Step42Control_GetPulseSpeedLimitForMicrosteps(motor_microsteps);
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

static uint32_t Program_CalcCrc(const Step42ProgramFlash_t *program)
{
  const uint32_t *words = (const uint32_t *)program;
  uint32_t crc = 0x3C6EF372UL;
  uint32_t count = (sizeof(Step42ProgramFlash_t) / sizeof(uint32_t)) - 1U;
  uint32_t i;

  for (i = 0U; i < count; i++)
  {
    crc ^= words[i] + 0x9E3779B9UL + (crc << 6) + (crc >> 2);
  }
  return crc;
}

static void Program_Load(void)
{
  const Step42ProgramFlash_t *stored =
      (const Step42ProgramFlash_t *)STEP42_PROGRAM_CFG_ADDR;

  program_count = 0U;
  program_ready = 0U;
  program_editing = 0U;
  program_save_armed = 0U;
  if ((stored->magic != STEP42_PROGRAM_MAGIC) ||
      (stored->version != STEP42_PROGRAM_VERSION) ||
      (stored->count > STEP42_PROGRAM_MAX_STEPS) ||
      (stored->crc != Program_CalcCrc(stored)))
  {
    return;
  }

  program_count = (uint8_t)stored->count;
  memcpy(program_steps, stored->steps, sizeof(program_steps));
  program_ready = 1U;
}

static void Config_Load(void)
{
  const Step42FlashConfig_t *cfg = (const Step42FlashConfig_t *)STEP42_CFG_ADDR;
  const Step42FlashConfigV2_t *legacy_v2 = (const Step42FlashConfigV2_t *)STEP42_CFG_ADDR;
  const Step42FlashConfigV1_t *legacy = (const Step42FlashConfigV1_t *)STEP42_CFG_ADDR;

  loaded_flash_config_valid = 0U;

  if ((cfg->magic == STEP42_CFG_MAGIC) &&
      (cfg->version == STEP42_CFG_VERSION) &&
      (cfg->crc == Config_CalcCrc(cfg)))
  {
    zero_valid = (cfg->zero_valid != 0UL) ? 1U : 0U;
    zero_offset_deg = Normalize360(((float)cfg->zero_cdeg) / 100.0f);
    home_mode = (cfg->home_mode <= CTRL_HOME_MULTI_TURN) ?
                (uint8_t)cfg->home_mode : CTRL_HOME_NEAREST;
    can_node_id = (((uint8_t)cfg->reserved >= STEP42_CAN_NODE_ID_MIN) &&
                   ((uint8_t)cfg->reserved <= STEP42_CAN_NODE_ID_MAX)) ?
                      (uint8_t)cfg->reserved : STEP42_CAN_NODE_ID_MIN;
    memcpy(&loaded_flash_config, cfg, sizeof(loaded_flash_config));
    loaded_flash_config_valid = 1U;
    return;
  }

  if ((legacy_v2->magic == STEP42_CFG_MAGIC) &&
      (legacy_v2->version == 2UL) &&
      (legacy_v2->crc == ConfigV2_CalcCrc(legacy_v2)))
  {
    zero_valid = (legacy_v2->zero_valid != 0UL) ? 1U : 0U;
    zero_offset_deg = Normalize360(((float)legacy_v2->zero_cdeg) / 100.0f);
    home_mode = (legacy_v2->home_mode <= CTRL_HOME_MULTI_TURN) ?
                (uint8_t)legacy_v2->home_mode : CTRL_HOME_NEAREST;
    can_node_id = (((uint8_t)legacy_v2->reserved >= STEP42_CAN_NODE_ID_MIN) &&
                   ((uint8_t)legacy_v2->reserved <= STEP42_CAN_NODE_ID_MAX)) ?
                      (uint8_t)legacy_v2->reserved : STEP42_CAN_NODE_ID_MIN;
    memset(&loaded_flash_config, 0, sizeof(loaded_flash_config));
    loaded_flash_config.magic = legacy_v2->magic;
    loaded_flash_config.version = STEP42_CFG_VERSION;
    loaded_flash_config.zero_valid = legacy_v2->zero_valid;
    loaded_flash_config.zero_cdeg = legacy_v2->zero_cdeg;
    loaded_flash_config.home_mode = legacy_v2->home_mode;
    loaded_flash_config.tracking = legacy_v2->tracking;
    loaded_flash_config.tracking_reverse = legacy_v2->tracking_reverse;
    loaded_flash_config.reserved = legacy_v2->reserved;
    loaded_flash_config.external_position = external_position_config;
    loaded_flash_config_valid = 1U;
    return;
  }

  /* Preserve zero-point data written by firmware v1, even though PID data
     did not exist in that old image. */
  if ((legacy->magic == STEP42_CFG_MAGIC) &&
      (legacy->version == 1UL) &&
      (legacy->crc == ConfigV1_CalcCrc(legacy)))
  {
    zero_valid = (legacy->zero_valid != 0UL) ? 1U : 0U;
    zero_offset_deg = Normalize360(((float)legacy->zero_cdeg) / 100.0f);
    home_mode = (legacy->home_mode <= CTRL_HOME_MULTI_TURN) ?
                (uint8_t)legacy->home_mode : CTRL_HOME_NEAREST;
    can_node_id = STEP42_CAN_NODE_ID_MIN;
  }
  else
  {
    zero_valid = 0U;
    zero_offset_deg = 0.0f;
    home_mode = CTRL_HOME_NEAREST;
    can_node_id = STEP42_CAN_NODE_ID_MIN;
  }
}

static void Config_ApplyRuntime(void)
{
  const Step42TrackingConfig_t *tracking;

  if (loaded_flash_config_valid == 0U)
  {
    return;
  }

  if (Step42Control_SetExternalPositionConfig(
          &loaded_flash_config.external_position) == 0U)
  {
    ExternalPosition_ResetDefaults();
  }

  /* Closed-position tuning is deliberately session-only.  Every boot starts
     with the known 32-microstep defaults, keeping PWM behavior predictable. */
  tracking = &loaded_flash_config.tracking;
  if (Step42Tracking_SetBasic(tracking->mode,
                               tracking->speed_percent,
                               tracking->accel_percent,
                               tracking->frequency_khz) == 0U)
  {
    return;
  }
  (void)Step42Tracking_SetPid(STEP42_TRACK_LOOP_SINGLE,
                               tracking->single.kp, tracking->single.ki, tracking->single.kd);
  (void)Step42Tracking_SetPid(STEP42_TRACK_LOOP_OUTER,
                               tracking->outer.kp, tracking->outer.ki, tracking->outer.kd);
  (void)Step42Tracking_SetPid(STEP42_TRACK_LOOP_INNER,
                               tracking->inner.kp, tracking->inner.ki, tracking->inner.kd);
  (void)Step42Tracking_SetAdvanced(tracking->filter_alpha,
                                    tracking->dynamic_gain_min,
                                    tracking->dynamic_gain_max,
                                    tracking->dynamic_full_error,
                                    tracking->prediction_ms,
                                    tracking->timeout_ms,
                                    tracking->single.deadband,
                                    tracking->outer.deadband,
                                    tracking->inner.deadband);
  tracking_error_reverse = (loaded_flash_config.tracking_reverse != 0UL) ? 1U : 0U;
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
  const Step42TrackingConfig_t *tracking = Step42Tracking_GetConfig();

  if ((last_flash_write_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - last_flash_write_ms) < STEP42_FLASH_WRITE_GUARD_MS))
  {
    return CTRL_STATUS_BUSY;
  }

  memset(&image, 0, sizeof(image));
  image.cfg.magic = STEP42_CFG_MAGIC;
  image.cfg.version = STEP42_CFG_VERSION;
  image.cfg.zero_valid = zero_valid;
  image.cfg.zero_cdeg = (int32_t)(zero_offset_deg * 100.0f + 0.5f);
  image.cfg.home_mode = home_mode;
  /* Position PID and its acceleration are intentionally not persisted. */
  image.cfg.tracking = *tracking;
  image.cfg.tracking_reverse = tracking_error_reverse;
  image.cfg.reserved = can_node_id;
  image.cfg.external_position = external_position_config;
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
  memcpy(&loaded_flash_config, &image.cfg, sizeof(loaded_flash_config));
  loaded_flash_config_valid = 1U;
  last_flash_write_ms = HAL_GetTick();
  return CTRL_STATUS_OK;
}

uint8_t Step42Control_SetProgramStep(uint8_t index, const Step42ProgramStep_t *step)
{
  if ((step == 0) || (index >= STEP42_PROGRAM_MAX_STEPS) ||
      (step->op < STEP42_PROGRAM_OP_POSITION_ABS) ||
      (step->op > STEP42_PROGRAM_OP_BRANCH))
  {
    return CTRL_STATUS_RANGE;
  }

  if ((step->condition > STEP42_PROGRAM_CONDITION_SIGNAL_FALSE) ||
      ((step->condition != STEP42_PROGRAM_CONDITION_NONE) &&
       ((step->signal > STEP42_PROGRAM_SIGNAL_CUSTOM2) ||
        (step->jump_index >= STEP42_PROGRAM_MAX_STEPS))))
  {
    return CTRL_STATUS_RANGE;
  }

  if ((step->op == STEP42_PROGRAM_OP_MICROSTEP) &&
      (step->value != 1) && (step->value != 2) && (step->value != 4) &&
      (step->value != 8) && (step->value != 16) && (step->value != 32) &&
      (step->value != 64) && (step->value != 128) && (step->value != 256))
  {
    return CTRL_STATUS_RANGE;
  }
  if ((step->op == STEP42_PROGRAM_OP_CHOPPER) && (step->flags > 1U))
  {
    return CTRL_STATUS_RANGE;
  }
  if ((step->op == STEP42_PROGRAM_OP_HOME) && (step->flags > CTRL_HOME_MULTI_TURN))
  {
    return CTRL_STATUS_RANGE;
  }
  if (((step->op == STEP42_PROGRAM_OP_POSITION_ABS) ||
       (step->op == STEP42_PROGRAM_OP_POSITION_REL) ||
       (step->op == STEP42_PROGRAM_OP_OPEN_STEPS)) &&
      ((step->speed == 0U) || (step->speed > STEP42_HARD_MAX_PULSE_SPEED) ||
       (step->accel == 0U)))
  {
    return CTRL_STATUS_RANGE;
  }
  if (step->op == STEP42_PROGRAM_OP_DRIVE_STATE)
  {
    if ((step->flags & (uint8_t)~0x0FU) != 0U)
    {
      return CTRL_STATUS_RANGE;
    }
    if (((step->flags & STEP42_PROGRAM_STATE_MODE) != 0U) &&
        ((step->value < CTRL_MODE_OPEN_POSITION) ||
         (step->value > CTRL_MODE_EXTERNAL_POSITION)))
    {
      return CTRL_STATUS_RANGE;
    }
    if (((step->flags & STEP42_PROGRAM_STATE_MICROSTEP) != 0U) &&
        (step->speed != 1U) && (step->speed != 2U) &&
        (step->speed != 4U) && (step->speed != 8U) &&
        (step->speed != 16U) && (step->speed != 32U) &&
        (step->speed != 64U) && (step->speed != 128U) &&
        (step->speed != 256U))
    {
      return CTRL_STATUS_RANGE;
    }
    if ((((step->flags & STEP42_PROGRAM_STATE_CHOPPER) != 0U) &&
         (step->accel > 1U)) ||
        (((step->flags & STEP42_PROGRAM_STATE_PROFILE) != 0U) &&
         (step->duration_ms > 1U)))
    {
      return CTRL_STATUS_RANGE;
    }
  }
  if ((step->op == STEP42_PROGRAM_OP_BRANCH) &&
      (step->condition == STEP42_PROGRAM_CONDITION_NONE))
  {
    return CTRL_STATUS_RANGE;
  }
  if ((step->op == STEP42_PROGRAM_OP_GOTO) &&
      ((step->value < 0) || (step->value >= (int32_t)STEP42_PROGRAM_MAX_STEPS)))
  {
    return CTRL_STATUS_RANGE;
  }

  if (program_running != 0U)
  {
    return CTRL_STATUS_BUSY;
  }

  program_steps[index] = *step;
  program_editing = 1U;
  program_save_armed = 0U;
  return CTRL_STATUS_OK;
}

uint8_t Step42Control_StageProgram(uint8_t count)
{
  uint8_t i;

  if ((count == 0U) || (count > STEP42_PROGRAM_MAX_STEPS))
  {
    return CTRL_STATUS_RANGE;
  }
  if (program_running != 0U)
  {
    return CTRL_STATUS_BUSY;
  }

  for (i = 0U; i < count; i++)
  {
    const Step42ProgramStep_t *step = &program_steps[i];

    if (((step->condition != STEP42_PROGRAM_CONDITION_NONE) &&
         (step->jump_index >= count)) ||
        ((step->op == STEP42_PROGRAM_OP_GOTO) &&
         ((step->value < 0) || (step->value >= count))))
    {
      return CTRL_STATUS_RANGE;
    }
  }

  program_count = count;
  program_ready = 1U;
  program_editing = 0U;
  program_save_armed = 1U;
  return CTRL_STATUS_OK;
}

uint8_t Step42Control_SaveProgram(uint8_t count)
{
  union
  {
    Step42ProgramFlash_t program;
    uint64_t words[sizeof(Step42ProgramFlash_t) / sizeof(uint64_t)];
  } image;
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0U;
  uint32_t address = STEP42_PROGRAM_CFG_ADDR;
  uint32_t i;

  if (count > STEP42_PROGRAM_MAX_STEPS)
  {
    return CTRL_STATUS_RANGE;
  }
  if ((program_save_armed == 0U) || (program_editing != 0U) ||
      (count != program_count))
  {
    return CTRL_STATUS_BAD_MODE;
  }
  if ((program_last_flash_write_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - program_last_flash_write_ms) < STEP42_FLASH_WRITE_GUARD_MS))
  {
    return CTRL_STATUS_BUSY;
  }

  image.program.magic = STEP42_PROGRAM_MAGIC;
  image.program.version = STEP42_PROGRAM_VERSION;
  image.program.count = count;
  memcpy(image.program.steps, program_steps, sizeof(program_steps));
  image.program.crc = Program_CalcCrc(&image.program);

  Step42Control_StopMotion();
  HAL_FLASH_Unlock();
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = STEP42_PROGRAM_FLASH_PAGE;
  erase.NbPages = 1U;
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return CTRL_STATUS_RANGE;
  }

  for (i = 0U; i < (sizeof(Step42ProgramFlash_t) / sizeof(uint64_t)); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, image.words[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      return CTRL_STATUS_RANGE;
    }
    address += 8U;
  }
  HAL_FLASH_Lock();
  program_count = count;
  program_ready = 1U;
  program_editing = 0U;
  program_save_armed = 0U;
  program_last_flash_write_ms = HAL_GetTick();
  return CTRL_STATUS_OK;
}

static void Program_CaptureRuntime(void)
{
  program_snapshot.valid = 1U;
  program_snapshot.mode = control_mode;
  program_snapshot.user_enabled = motor_user_enabled;
  program_snapshot.chopper_mode = Tmc2209_GetChopperMode();
  program_snapshot.microsteps = motor_microsteps;
  program_snapshot.home_mode = home_mode;
  program_snapshot.closed_accel = closed_position_accel;
  Step42Control_GetPositionPID(&program_snapshot.kp,
                               &program_snapshot.ki,
                               &program_snapshot.kd,
                               &program_snapshot.pid_limit,
                               &program_snapshot.deadband);
}

static void Program_RestoreRuntime(void)
{
  Step42ProgramSnapshot_t snapshot = program_snapshot;

  if (snapshot.valid == 0U)
  {
    Step42Control_StopMotion();
    return;
  }

  Step42Control_StopMotion();
  (void)Step42Control_SetMode(snapshot.mode);
  if ((snapshot.mode != CTRL_MODE_TRACK_PID) &&
      (snapshot.mode != CTRL_MODE_EXTERNAL_POSITION) &&
      (motor_microsteps != snapshot.microsteps))
  {
    (void)Step42Control_SetMicrosteps(snapshot.microsteps);
  }
  (void)Tmc2209_SetChopperMode(snapshot.chopper_mode);
  closed_position_accel = snapshot.closed_accel;
  Step42Control_SetPositionPID(snapshot.kp, snapshot.ki, snapshot.kd,
                                snapshot.pid_limit, snapshot.deadband);
  home_mode = snapshot.home_mode;
  Step42Control_SetUserEnable(snapshot.user_enabled);
  program_snapshot.valid = 0U;
}

uint8_t Step42Control_SetProgramMode(uint8_t enable)
{
  if (enable == 0U)
  {
    program_running = 0U;
    Program_RestoreRuntime();
    program_mode = 0U;
    return CTRL_STATUS_OK;
  }

  if (program_mode != 0U)
  {
    return CTRL_STATUS_OK;
  }

  Program_CaptureRuntime();
  Step42Control_StopMotion();
  program_mode = 1U;
  return CTRL_STATUS_OK;
}

uint8_t Step42Control_IsProgramMode(void)
{
  return program_mode;
}

uint8_t Step42Control_IsProgramRunning(void)
{
  return program_running;
}

uint8_t Step42Control_GetProgramCount(void)
{
  return program_count;
}

static uint8_t Program_Start(uint8_t require_program_mode)
{
  if (((require_program_mode != 0U) && (program_mode == 0U)) ||
      (program_ready == 0U) ||
      (program_editing != 0U) || (program_count == 0U) ||
      (motor_user_enabled == 0U) || (thermal_lockout != 0U) || (stall_fault != 0U))
  {
    return CTRL_STATUS_BAD_MODE;
  }

  Step42Control_StopMotion();
  program_index = 0U;
  program_step_started = 0U;
  program_running = 1U;
  return CTRL_STATUS_OK;
}

uint8_t Step42Control_StartProgram(void)
{
  return Program_Start(1U);
}

uint8_t Step42Control_StartOfflineProgram(void)
{
  return Program_Start(0U);
}

static void Program_Advance(void)
{
  program_index++;
  program_step_started = 0U;
  if (program_index >= program_count)
  {
    program_running = 0U;
    Step42Control_StopMotion();
  }
}

static uint8_t Program_ConditionMet(const Step42ProgramStep_t *step)
{
  uint8_t signal_value;

  if ((step == 0) || (step->condition == STEP42_PROGRAM_CONDITION_NONE))
  {
    return 0U;
  }
  if (step->signal == STEP42_PROGRAM_SIGNAL_TARGET)
  {
    signal_value = program_target_detected;
  }
  else if (step->signal == STEP42_PROGRAM_SIGNAL_CUSTOM1)
  {
    signal_value = program_custom_signal1;
  }
  else
  {
    signal_value = program_custom_signal2;
  }

  if (step->condition == STEP42_PROGRAM_CONDITION_SIGNAL_TRUE)
  {
    return (signal_value != 0U) ? 1U : 0U;
  }
  return (signal_value == 0U) ? 1U : 0U;
}

static void Program_Jump(uint8_t index)
{
  if (index >= program_count)
  {
    program_running = 0U;
    Step42Control_StopMotion();
    return;
  }

  Step42Control_StopMotion();
  program_index = index;
  program_step_started = 0U;
}

static uint8_t Program_ApplyDriveState(const Step42ProgramStep_t *step)
{
  uint8_t target_mode = control_mode;

  if ((step->flags & STEP42_PROGRAM_STATE_MODE) != 0U)
  {
    target_mode = (uint8_t)step->value;
  }
  if (((step->flags & STEP42_PROGRAM_STATE_MICROSTEP) != 0U) &&
      (target_mode == CTRL_MODE_TRACK_PID) && (step->speed != 32U))
  {
    return 0U;
  }
  if (((step->flags & STEP42_PROGRAM_STATE_MICROSTEP) != 0U) &&
      ((control_mode == CTRL_MODE_TRACK_PID) ||
       (control_mode == CTRL_MODE_EXTERNAL_POSITION)))
  {
    if (Step42Control_SetMode(CTRL_MODE_OPEN_POSITION) == 0U)
    {
      return 0U;
    }
  }
  if (((step->flags & STEP42_PROGRAM_STATE_MICROSTEP) != 0U) &&
      (Step42Control_SetMicrosteps(step->speed) == 0U))
  {
    return 0U;
  }
  if (((step->flags & STEP42_PROGRAM_STATE_CHOPPER) != 0U) &&
      (Tmc2209_SetChopperMode((uint8_t)step->accel) == 0U))
  {
    return 0U;
  }
  if (((step->flags & STEP42_PROGRAM_STATE_PROFILE) != 0U) &&
      (Step42Control_SetMotionProfile((uint8_t)step->duration_ms) == 0U))
  {
    return 0U;
  }
  if ((control_mode != target_mode) &&
      (Step42Control_SetMode(target_mode) == 0U))
  {
    return 0U;
  }
  return 1U;
}

void Step42Control_ProcessProgram(void)
{
  Step42ProgramStep_t *step;
  uint32_t elapsed;

  if (program_running == 0U)
  {
    return;
  }
  if ((motor_user_enabled == 0U) || (thermal_lockout != 0U) || (stall_fault != 0U))
  {
    program_running = 0U;
    Step42Control_StopMotion();
    return;
  }

  step = &program_steps[program_index];
  elapsed = (uint32_t)(HAL_GetTick() - program_step_start_ms);
  if ((step->op != STEP42_PROGRAM_OP_BRANCH) &&
      (Program_ConditionMet(step) != 0U))
  {
    Program_Jump(step->jump_index);
    return;
  }
  if (program_step_started == 0U)
  {
    program_step_started = 1U;
    program_step_start_ms = HAL_GetTick();
    elapsed = 0U;

    switch (step->op)
    {
    case STEP42_PROGRAM_OP_POSITION_ABS:
    {
      float kp;
      float ki;
      float kd;
      float limit;
      float deadband;

      if (step->speed > Step42Control_GetPulseSpeedLimit())
      {
        program_running = 0U;
        Step42Control_StopMotion();
        return;
      }

      (void)Step42Control_SetMode(CTRL_MODE_CLOSED_POSITION);
      (void)Step42Control_SetClosedAccel((float)step->accel);
      Step42Control_GetPositionPID(&kp, &ki, &kd, &limit, &deadband);
      Step42Control_SetPositionPID(kp, ki, kd, (float)step->speed, deadband);
      Step42Control_MoveToAngleByMode(((float)step->value) / 100.0f);
      break;
    }
    case STEP42_PROGRAM_OP_POSITION_REL:
    {
      float kp;
      float ki;
      float kd;
      float limit;
      float deadband;

      if (step->speed > Step42Control_GetPulseSpeedLimit())
      {
        program_running = 0U;
        Step42Control_StopMotion();
        return;
      }

      (void)Step42Control_SetMode(CTRL_MODE_CLOSED_POSITION);
      (void)Step42Control_SetClosedAccel((float)step->accel);
      Step42Control_GetPositionPID(&kp, &ki, &kd, &limit, &deadband);
      Step42Control_SetPositionPID(kp, ki, kd, (float)step->speed, deadband);
      (void)Step42Control_MoveRelativeAngleByMode(((float)step->value) / 100.0f);
      break;
    }
    case STEP42_PROGRAM_OP_OPEN_STEPS:
      if (step->speed > Step42Control_GetPulseSpeedLimit())
      {
        program_running = 0U;
        Step42Control_StopMotion();
        return;
      }
      (void)Step42Control_SetMode(CTRL_MODE_OPEN_POSITION);
      (void)Step42Control_MoveStepsByMode(step->value, (float)step->speed, (float)step->accel);
      break;
    case STEP42_PROGRAM_OP_SPEED:
    {
      float speed = (float)step->value;

      if ((step->flags & STEP42_PROGRAM_SPEED_FLAG_RPM) != 0U)
      {
        speed = speed * motor_steps_per_rev / 60.0f;
      }
      if ((AbsF(speed) > (float)Step42Control_GetPulseSpeedLimit()) ||
          (step->accel == 0U))
      {
        program_running = 0U;
        Step42Control_StopMotion();
        return;
      }
      (void)Step42Control_SetMode(CTRL_MODE_OPEN_SPEED);
      Step42Control_RunSpeedByMode(speed, (float)step->accel);
      break;
    }
    case STEP42_PROGRAM_OP_HOME:
      (void)Step42Control_SetMode(CTRL_MODE_CLOSED_POSITION);
      (void)Step42Control_Home(step->flags);
      break;
    case STEP42_PROGRAM_OP_TRACK:
    {
      const Step42TrackingConfig_t *tracking = Step42Control_GetTrackingConfig();
      uint8_t tracking_mode = ((step->flags & 0x01U) != 0U) ?
                              STEP42_TRACK_CASCADE : STEP42_TRACK_SINGLE;

      /* The program owns the motion mode, while UART continues to provide
         live visual error samples to this tracking controller. */
      (void)Step42Control_SetTrackingBasic(tracking_mode,
                                            tracking->speed_percent,
                                            tracking->accel_percent,
                                            tracking->frequency_khz);
      (void)Step42Control_SetMode(CTRL_MODE_TRACK_PID);
      Step42Control_SetTrackingInput(((float)step->value) / 100.0f, 0.0f);
      break;
    }
    case STEP42_PROGRAM_OP_EXTERNAL_POSITION:
      if (Step42Control_SetMode(CTRL_MODE_EXTERNAL_POSITION) == 0U)
      {
        program_running = 0U;
        Step42Control_StopMotion();
        return;
      }
      break;
    case STEP42_PROGRAM_OP_STOP:
      Step42Control_StopMotion();
      break;
    case STEP42_PROGRAM_OP_MICROSTEP:
      if (control_mode == CTRL_MODE_TRACK_PID)
      {
        (void)Step42Control_SetMode(CTRL_MODE_OPEN_POSITION);
      }
      (void)Step42Control_SetMicrosteps((uint16_t)step->value);
      break;
    case STEP42_PROGRAM_OP_CHOPPER:
      (void)Tmc2209_SetChopperMode(step->flags);
      break;
    case STEP42_PROGRAM_OP_DRIVE_STATE:
      if (Program_ApplyDriveState(step) == 0U)
      {
        program_running = 0U;
        Step42Control_StopMotion();
        return;
      }
      break;
    case STEP42_PROGRAM_OP_DELAY:
      /* Completion is handled below after duration_ms has elapsed. */
      break;
    case STEP42_PROGRAM_OP_GOTO:
      Program_Jump((uint8_t)step->value);
      return;
    case STEP42_PROGRAM_OP_BRANCH:
      if (Program_ConditionMet(step) != 0U)
      {
        Program_Jump(step->jump_index);
      }
      else
      {
        Program_Advance();
      }
      return;
    default:
      program_running = 0U;
      Step42Control_StopMotion();
      return;
    }
  }

  if (step->op == STEP42_PROGRAM_OP_DELAY)
  {
    if (elapsed >= step->duration_ms)
    {
      Program_Advance();
    }
  }
  else if ((step->op == STEP42_PROGRAM_OP_SPEED) ||
           (step->op == STEP42_PROGRAM_OP_TRACK) ||
           (step->op == STEP42_PROGRAM_OP_EXTERNAL_POSITION))
  {
    if ((step->duration_ms != 0U) && (elapsed >= step->duration_ms))
    {
      Step42Control_StopMotion();
      Program_Advance();
    }
  }
  else if ((step->op == STEP42_PROGRAM_OP_STOP) ||
           (step->op == STEP42_PROGRAM_OP_MICROSTEP) ||
           (step->op == STEP42_PROGRAM_OP_CHOPPER) ||
           (step->op == STEP42_PROGRAM_OP_DRIVE_STATE))
  {
    Program_Advance();
  }
  else if ((step->op != STEP42_PROGRAM_OP_DELAY) &&
           (Step42Control_IsMotionActive() == 0U))
  {
    Program_Advance();
  }
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

static void Encoder_RebaseUnwrapped(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (mt6701_unwrap_valid != 0U)
  {
    mt6701_unwrapped_deg = mt6701_angle_deg;
    mt6701_last_raw = mt6701_raw_angle;
  }
  speed_feedback_valid = 0U;
  speed_accum_delta_deg = 0.0f;
  speed_accum_time_s = 0.0f;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void Encoder_RebaseFromTimer(void)
{
  mt6701_unwrapped_deg = mt6701_angle_deg;
  mt6701_last_raw = mt6701_raw_angle;
  speed_feedback_valid = 0U;
  speed_accum_delta_deg = 0.0f;
  speed_accum_time_s = 0.0f;
}

static void Motor_EnableOutput(void)
{
  if (thermal_lockout == 0U)
  {
    Stepper_EN(tmc_EN_GPIO_Port, tmc_EN_Pin, TMC_EN);
  }
}

static void Motor_DisableOutput(void)
{
  weak_pid_enable = 0U;
  Step42Tracking_Stop();
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
  if ((output > 0.0f) && (output < WEAK_PID_MIN_CORRECTION_SPEED))
  {
    output = WEAK_PID_MIN_CORRECTION_SPEED;
  }
  else if ((output < 0.0f) && (output > -WEAK_PID_MIN_CORRECTION_SPEED))
  {
    output = -WEAK_PID_MIN_CORRECTION_SPEED;
  }
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

static void ExternalPosition_ResetDefaults(void)
{
  memset(&external_position_config, 0, sizeof(external_position_config));
  external_position_config.microsteps = 32U;
  external_position_config.accel = 3000U;
  external_position_config.kp = 180.0f;
  external_position_config.ki = 0.2f;
  external_position_config.kd = 0.0f;
  external_position_config.output_limit = 5000.0f;
  external_position_config.deadband = 0.08f;
  external_position_config.minimum_deg = -80.0f;
  external_position_config.maximum_deg = 80.0f;
  external_position_config.timeout_ms = 200U;
}

static void ExternalPosition_ApplyConfig(void)
{
  closed_position_accel = (float)external_position_config.accel;
  PID_Init(&weak_pid,
           external_position_config.kp,
           external_position_config.ki,
           external_position_config.kd,
           0.001f,
           external_position_config.output_limit,
           20.0f,
           external_position_config.deadband);
}

static void ExternalPosition_CaptureRuntime(void)
{
  external_runtime_snapshot.valid = 1U;
  external_runtime_snapshot.microsteps = motor_microsteps;
  external_runtime_snapshot.closed_accel = closed_position_accel;
  Step42Control_GetPositionPID(&external_runtime_snapshot.kp,
                               &external_runtime_snapshot.ki,
                               &external_runtime_snapshot.kd,
                               &external_runtime_snapshot.pid_limit,
                               &external_runtime_snapshot.deadband);
}

static void ExternalPosition_RestoreRuntime(void)
{
  Step42ExternalRuntimeSnapshot_t snapshot = external_runtime_snapshot;

  if (snapshot.valid == 0U)
  {
    return;
  }

  if ((motor_microsteps != snapshot.microsteps) &&
      (Tmc2209_SetMicrosteps(snapshot.microsteps) != 0U))
  {
    motor_microsteps = snapshot.microsteps;
    motor_steps_per_rev = MOTOR_BASE_STEPS_PER_REV * (float)motor_microsteps;
  }
  closed_position_accel = snapshot.closed_accel;
  Step42Control_SetPositionPID(snapshot.kp, snapshot.ki, snapshot.kd,
                               snapshot.pid_limit, snapshot.deadband);
  /* External mode uses its own signed coordinate. Resume ordinary closed
     position from the current physical single-turn cycle, never an old
     accumulated count from before the external session. */
  Encoder_RebaseUnwrapped();
  external_runtime_snapshot.valid = 0U;
}

static void ExternalPosition_Rebase(void)
{
  float physical_deg = PID_RawToDeg(mt6701_raw_angle);

  if (external_position_config.center_valid == 0U)
  {
    external_position_unwrap_valid = 0U;
    external_position_current_deg = 0.0f;
    return;
  }

  /* The initial point is signed relative to the saved center. This recovers
     the same -80..+80 coordinate after a power cycle despite MT6701 only
     providing a single-turn absolute angle. */
  external_position_current_deg = PID_AngleErrorDeg(
      physical_deg, external_position_config.center_raw_deg);
  external_position_last_raw = mt6701_raw_angle;
  external_position_unwrap_valid = 1U;
}

static float ExternalPosition_UpdateCurrent(uint16_t raw_angle)
{
  int32_t delta_raw;

  if (external_position_unwrap_valid == 0U)
  {
    ExternalPosition_Rebase();
    return external_position_current_deg;
  }

  delta_raw = (int32_t)raw_angle - (int32_t)external_position_last_raw;
  if (delta_raw > 8192)
  {
    delta_raw -= 16384;
  }
  else if (delta_raw < -8192)
  {
    delta_raw += 16384;
  }

  external_position_last_raw = raw_angle;
  external_position_current_deg += ((float)delta_raw) * 360.0f / 16384.0f;
  return external_position_current_deg;
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

  /* Multi-turn absolute coordinates are referenced to the stored zero point. */
  weak_pid_target_deg = target_angle;
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
  float speed_limit = (float)Step42Control_GetPulseSpeedLimit();

  speed = ClampF(speed, -speed_limit, speed_limit);
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
  float speed_limit = (float)Step42Control_GetPulseSpeedLimit();

  speed = ClampF(speed, -speed_limit, speed_limit);
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
  float speed_limit = (float)Step42Control_GetPulseSpeedLimit();

  if ((motor_user_enabled == 0U) ||
      (AbsF(closed_speed_target_steps) < 1.0f) ||
      (speed_feedback_valid == 0U))
  {
    return;
  }

  correction = PID_UpdateLinear(&speed_pid, closed_speed_target_steps, closed_speed_feedback_steps);
  command_speed = closed_speed_target_steps + correction;
  if (command_speed > speed_limit)
  {
    command_speed = speed_limit;
  }
  else if (command_speed < -speed_limit)
  {
    command_speed = -speed_limit;
  }

  Stepper_Set_Speed_Profile(&Motor_42, command_speed, WEAK_PID_ACCEL_STEPS);
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
}

static void SetControlLoopFrequency(uint8_t frequency_khz)
{
  uint32_t new_hz = (uint32_t)frequency_khz * 1000UL;
  uint32_t period;

  if ((new_hz < 1000UL) || (new_hz > 10000UL))
  {
    new_hz = 1000UL;
  }
  period = (CONTROL_TIMER_TICK_HZ / new_hz) - 1UL;

  __disable_irq();
  __HAL_TIM_SET_AUTORELOAD(&htim6, period);
  __HAL_TIM_SET_COUNTER(&htim6, 0UL);
  __enable_irq();

  control_loop_hz = new_hz;
  weak_pid.dt = 1.0f / (float)control_loop_hz;
  speed_pid.dt = weak_pid.dt;
  speed_feedback_valid = 0U;
  speed_accum_delta_deg = 0.0f;
  speed_accum_time_s = 0.0f;
}

static void Tracking_ApplySpeed(float speed)
{
  if ((motor_user_enabled == 0U) || (thermal_lockout != 0U))
  {
    Stepper_ForceStopPosition(&Motor_42);
    return;
  }

  weak_pid_output_steps = speed;
  if (AbsF(speed) < 1.0f)
  {
    Stepper_Stop(&Motor_42);
    led_white_off;
    return;
  }

  if ((Motor_42.state != STOP) &&
      (((Motor_42.current_speed > 0.0f) && (speed < 0.0f)) ||
       ((Motor_42.current_speed < 0.0f) && (speed > 0.0f))))
  {
    Stepper_Stop(&Motor_42);
  }

  Motor_42.run_mode = MODE_SPEED;
  Stepper_Set_Speed_Profile(&Motor_42, speed, Step42Tracking_GetAccel());
  if (Motor_42.state == STOP)
  {
    Stepper_Run_Speed(&Motor_42);
  }
  led_white_on;
}

void Step42Control_Init(void)
{
  Stepper_Init(&Motor_42, &htim1, TIM_CHANNEL_1, tmc_DIR_GPIO_Port, tmc_DIR_Pin, MODE_SPEED);
  ExternalPosition_ResetDefaults();
  Config_Load();
  Program_Load();
  ApplyDefaultParams(motor_microsteps);
  Tmc2209_SetMicrosteps(motor_microsteps);
  PID_Init(&speed_pid, 0.25f, 1.0f, 0.0f, 0.001f, 1500.0f, 1000.0f, 5.0f);
  Step42Tracking_Init();
  Config_ApplyRuntime();
  Stepper_EN(tmc_EN_GPIO_Port, tmc_EN_Pin, TMC_UNEN);
}

void Step42Control_SetUserEnable(uint8_t enable)
{
  if (enable != 0U)
  {
    stall_fault = 0U;
    stall_motion_age_ms = 0U;
    stall_elapsed_ms = 0U;
    stall_reverse_elapsed_ms = 0U;
    stall_command_sign = 0;
    if (thermal_lockout != 0U)
    {
      Motor_DisableOutput();
      motor_user_enabled = 0U;
      return;
    }
    Motor_EnableOutput();
    if (control_mode == CTRL_MODE_CLOSED_POSITION)
    {
      weak_pid_target_deg = mt6701_unwrapped_deg;
      PID_Reset(&weak_pid);
      weak_pid_enable = 1U;
    }
    else if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
    {
      ExternalPosition_Rebase();
      external_position_target_deg = ClampF(external_position_current_deg,
                                            external_position_config.minimum_deg,
                                            external_position_config.maximum_deg);
      weak_pid_target_deg = external_position_target_deg;
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
    program_running = 0U;
    Motor_DisableOutput();
    led_red_off;
    led_white_off;
    motor_user_enabled = 0U;
  }
}

uint8_t Step42Control_SetMode(uint8_t mode)
{
  uint8_t previous_mode = control_mode;

  if ((mode != CTRL_MODE_OPEN_SPEED) &&
      (mode != CTRL_MODE_OPEN_POSITION) &&
      (mode != CTRL_MODE_CLOSED_POSITION) &&
      (mode != CTRL_MODE_TRACK_PID) &&
      (mode != CTRL_MODE_EXTERNAL_POSITION))
  {
    return 0U;
  }
  if ((mode == CTRL_MODE_EXTERNAL_POSITION) &&
      (external_position_config.center_valid == 0U))
  {
    return 0U;
  }

  Step42Control_StopMotion();
  if ((previous_mode == CTRL_MODE_EXTERNAL_POSITION) &&
      (mode != CTRL_MODE_EXTERNAL_POSITION))
  {
    /* Leaving this dedicated mode restores the ordinary position session. */
    ExternalPosition_RestoreRuntime();
  }
  if (mode == CTRL_MODE_CLOSED_POSITION)
  {
    /* An explicit closed-position mode selection starts a fresh multi-turn
       coordinate session. This prevents a Factory Debug reconnect from
       inheriting an old accumulated turn count and chasing it for many turns. */
    Encoder_RebaseUnwrapped();
  }
  if (mode == CTRL_MODE_TRACK_PID)
  {
    if (motor_microsteps != 32U)
    {
      if (Tmc2209_SetMicrosteps(32U) == 0U)
      {
        return 0U;
      }
      motor_microsteps = 32U;
      motor_steps_per_rev = MOTOR_BASE_STEPS_PER_REV * 32.0f;
    }
  }
  else if (mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    if (previous_mode != CTRL_MODE_EXTERNAL_POSITION)
    {
      ExternalPosition_CaptureRuntime();
    }
    if (Tmc2209_SetMicrosteps(external_position_config.microsteps) == 0U)
    {
      if (previous_mode != CTRL_MODE_EXTERNAL_POSITION)
      {
        external_runtime_snapshot.valid = 0U;
      }
      return 0U;
    }
    motor_microsteps = external_position_config.microsteps;
    motor_steps_per_rev = MOTOR_BASE_STEPS_PER_REV * (float)motor_microsteps;
    ExternalPosition_ApplyConfig();
    ExternalPosition_Rebase();
    external_position_target_deg = ClampF(external_position_current_deg,
                                          external_position_config.minimum_deg,
                                          external_position_config.maximum_deg);
    external_position_last_target_ms = HAL_GetTick();
  }

  control_mode = mode;
  if (mode == CTRL_MODE_TRACK_PID)
  {
    SetControlLoopFrequency(Step42Tracking_GetConfig()->frequency_khz);
    Motor_42.run_mode = MODE_SPEED;
    Step42Tracking_Stop();
  }
  else
  {
    SetControlLoopFrequency(1U);
  }
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
  else if ((motor_user_enabled != 0U) && (mode == CTRL_MODE_EXTERNAL_POSITION))
  {
    weak_pid_target_deg = external_position_target_deg;
    PID_Reset(&weak_pid);
    weak_pid_enable = 1U;
  }
  return 1U;
}

void Step42Control_MoveToAngleByMode(float target_angle)
{
  if (control_mode == CTRL_MODE_CLOSED_POSITION)
  {
    Move_ToAbsoluteAngle(target_angle);
  }
}

void Step42Control_MoveToPwmAngle(float target_angle)
{
  float turns;
  int32_t nearest_turn;

  if (control_mode != CTRL_MODE_CLOSED_POSITION)
  {
    return;
  }

  /* PWM is a single-turn cyclic command: 0 deg and 360 deg are equivalent.
     Choose the closest equivalent target so crossing the zero point never
     creates an accidental near-full-turn correction. */
  target_angle = Normalize360(target_angle);
  turns = (mt6701_unwrapped_deg - target_angle) / 360.0f;
  nearest_turn = (turns >= 0.0f) ?
                 (int32_t)(turns + 0.5f) : (int32_t)(turns - 0.5f);
  Move_ToAbsoluteAngle(target_angle + ((float)nearest_turn * 360.0f));
}

uint8_t Step42Control_MoveRelativeAngleByMode(float delta_angle)
{
  float target_angle;

  if (control_mode != CTRL_MODE_CLOSED_POSITION)
  {
    return 0U;
  }

  target_angle = mt6701_unwrapped_deg + delta_angle;
  if ((target_angle < -CLOSED_POSITION_LIMIT_DEG) ||
      (target_angle > CLOSED_POSITION_LIMIT_DEG))
  {
    return 0U;
  }

  Move_ToAbsoluteAngle(target_angle);
  return 1U;
}

uint8_t Step42Control_MoveStepsByMode(int32_t steps, float speed, float accel)
{
  float delta_deg;

  if ((steps == 0) || (speed < 1.0f) ||
      (speed > (float)Step42Control_GetPulseSpeedLimit()) || (accel < 1.0f))
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
  Step42Tracking_Stop();
  closed_speed_target_steps = 0.0f;
  Stepper_ForceStopPosition(&Motor_42);
  external_task_marked = 0U;
  led_white_off;
}

void Step42Control_AbortProgram(void)
{
  program_running = 0U;
  Step42Control_StopMotion();
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

static uint32_t ConfigV1_CalcCrc(const Step42FlashConfigV1_t *cfg)
{
  const uint32_t *words = (const uint32_t *)cfg;
  uint32_t crc = 0xA5A55A5AUL;
  uint32_t count = (sizeof(Step42FlashConfigV1_t) / sizeof(uint32_t)) - 1U;
  uint32_t i;

  for (i = 0U; i < count; i++)
  {
    crc ^= words[i] + 0x9E3779B9UL + (crc << 6) + (crc >> 2);
  }

  return crc;
}

static uint32_t ConfigV2_CalcCrc(const Step42FlashConfigV2_t *cfg)
{
  const uint32_t *words = (const uint32_t *)cfg;
  uint32_t crc = 0xA5A55A5AUL;
  uint32_t count = (sizeof(Step42FlashConfigV2_t) / sizeof(uint32_t)) - 1U;
  uint32_t i;

  for (i = 0U; i < count; i++)
  {
    crc ^= words[i] + 0x9E3779B9UL + (crc << 6) + (crc >> 2);
  }
  return crc;
}

uint8_t Step42Control_SetMotionProfile(uint8_t profile)
{
  if (profile > (uint8_t)STEPPER_PROFILE_S_CURVE)
  {
    return 0U;
  }

  /* A profile change is configuration, never a live speed transition. */
  Step42Control_StopMotion();
  Stepper_SetMotionProfile(&Motor_42, (Stepper_Profile_t)profile);
  return 1U;
}

uint8_t Step42Control_SetMicrosteps(uint16_t microsteps)
{
  if ((control_mode == CTRL_MODE_TRACK_PID) ||
      (control_mode == CTRL_MODE_EXTERNAL_POSITION))
  {
    return 0U;
  }
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
  if ((control_mode == CTRL_MODE_CLOSED_POSITION) &&
      (motor_user_enabled != 0U))
  {
    weak_pid_target_deg = mt6701_unwrapped_deg;
    PID_Reset(&weak_pid);
    weak_pid_enable = 1U;
  }
  return 1U;
}

void Step42Control_SetPositionPID(float kp, float ki, float kd, float output_limit, float deadband)
{
  float speed_limit = (float)Step42Control_GetPulseSpeedLimit();

  if (output_limit > speed_limit)
  {
    output_limit = speed_limit;
  }
  PID_Init(&weak_pid, kp, ki, kd, 0.001f, output_limit, 20.0f, deadband);
}

uint8_t Step42Control_SaveRuntimeConfig(void)
{
  return Config_Save();
}

uint8_t Step42Control_SetCanNodeId(uint8_t node_id)
{
  uint8_t previous;
  uint8_t status;

  if ((node_id < STEP42_CAN_NODE_ID_MIN) || (node_id > STEP42_CAN_NODE_ID_MAX))
  {
    return CTRL_STATUS_RANGE;
  }
  if (node_id == can_node_id)
  {
    return CTRL_STATUS_OK;
  }

  previous = can_node_id;
  can_node_id = node_id;
  status = Config_Save();
  if (status != CTRL_STATUS_OK)
  {
    can_node_id = previous;
  }
  return status;
}

uint8_t Step42Control_GetCanNodeId(void)
{
  return can_node_id;
}

uint8_t Step42Control_SetHomeMode(uint8_t mode)
{
  uint8_t status;

  if (mode > CTRL_HOME_MULTI_TURN)
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
  float target;

  if (mode > CTRL_HOME_MULTI_TURN)
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
  else if (mode == CTRL_HOME_NEGATIVE)
  {
    delta = -mt6701_angle_deg;
  }
  else
  {
    delta = 0.0f;
  }

  if (motor_user_enabled == 0U)
  {
    Step42Control_SetUserEnable(1U);
  }

  target = (mode == CTRL_HOME_MULTI_TURN) ? 0.0f : (mt6701_unwrapped_deg + delta);
  weak_pid_target_deg = target;
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
  case KEY_EVENT_HOME_SHORT:
    if (motor_user_enabled != 0U)
    {
      (void)Step42Control_SetMode(CTRL_MODE_CLOSED_POSITION);
      (void)Step42Control_Home(home_mode);
    }
    break;

  case KEY_EVENT_ENABLE_SHORT:
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

  case KEY_EVENT_PROGRAM_LONG:
    if (motor_user_enabled != 0U)
    {
      (void)Step42Control_StartOfflineProgram();
    }
    break;

  case KEY_EVENT_STOP_LONG:
    Step42Control_AbortProgram();
    Step42Control_SetUserEnable(0U);
    break;

  default:
    break;
  }
}

uint8_t Step42Control_IsMotionActive(void)
{
  if (motor_user_enabled == 0U)
  {
    return 0U;
  }
  if (Motor_42.state != STOP)
  {
    return 1U;
  }
  if ((control_mode == CTRL_MODE_CLOSED_POSITION) &&
      (weak_pid_enable != 0U) &&
      (AbsF(Step42Control_GetPositionErrorDeg()) > weak_pid.deadband))
  {
    return 1U;
  }
  return 0U;
}

void Step42Control_MarkExternalTask(void)
{
  external_task_marked = 1U;
}

uint8_t Step42Control_IsExternalTaskActive(void)
{
  return ((external_task_marked != 0U) &&
          (program_running == 0U) &&
          (Step42Control_IsMotionActive() != 0U)) ? 1U : 0U;
}

uint8_t Step42Control_IsStallFault(void)
{
  return stall_fault;
}

static void StallProtection_Update(void)
{
  float commanded_steps_per_s;
  float commanded_rpm;
  float following_rpm;
  float feedback_limit_rpm;
  int8_t command_sign;
  uint8_t monitored_mode;

  monitored_mode = ((control_mode == CTRL_MODE_OPEN_SPEED) ||
                    (control_mode == CTRL_MODE_OPEN_POSITION) ||
                    (control_mode == CTRL_MODE_CLOSED_POSITION) ||
                    (control_mode == CTRL_MODE_TRACK_PID) ||
                    (control_mode == CTRL_MODE_EXTERNAL_POSITION)) ? 1U : 0U;
  if ((motor_user_enabled == 0U) || (monitored_mode == 0U) ||
      (Motor_42.state == STOP) || (motor_steps_per_rev < 1.0f))
  {
    stall_motion_age_ms = 0U;
    stall_elapsed_ms = 0U;
    stall_reverse_elapsed_ms = 0U;
    stall_command_sign = 0;
    return;
  }

  if (Motor_42.run_mode == MODE_POSITION)
  {
    commanded_steps_per_s = (Motor_42.step_delay > 0U) ?
                            ((float)Motor_42.timer_freq / (float)Motor_42.step_delay) : 0.0f;
    command_sign = (Motor_42.direction == DIR_REVERSE) ? 1 : -1;
  }
  else
  {
    commanded_steps_per_s = Motor_42.current_speed;
    command_sign = (commanded_steps_per_s >= 0.0f) ? 1 : -1;
  }
  commanded_rpm = AbsF(commanded_steps_per_s) * 60.0f / motor_steps_per_rev;

  if ((commanded_rpm < STALL_COMMAND_MIN_RPM) ||
      ((control_mode == CTRL_MODE_CLOSED_POSITION) &&
       (AbsF(Step42Control_GetPositionErrorDeg()) < STALL_POSITION_ERROR_DEG)))
  {
    stall_motion_age_ms = 0U;
    stall_elapsed_ms = 0U;
    stall_reverse_elapsed_ms = 0U;
    stall_command_sign = 0;
    return;
  }

  /* A requested direction change starts a fresh grace window. This avoids
     treating the rotor's normal deceleration as reverse motion. */
  if (command_sign != stall_command_sign)
  {
    stall_command_sign = command_sign;
    stall_motion_age_ms = 0U;
    stall_elapsed_ms = 0U;
    stall_reverse_elapsed_ms = 0U;
  }

  if (stall_motion_age_ms < STALL_STARTUP_GRACE_MS)
  {
    stall_motion_age_ms++;
    stall_elapsed_ms = 0U;
    stall_reverse_elapsed_ms = 0U;
    return;
  }

  feedback_limit_rpm = commanded_rpm * STALL_FEEDBACK_RATIO;
  if (feedback_limit_rpm < STALL_MIN_FEEDBACK_RPM)
  {
    feedback_limit_rpm = STALL_MIN_FEEDBACK_RPM;
  }

  /* Positive means the encoder is following the commanded direction;
     negative means the shaft is moving in the opposite direction. */
  following_rpm = motor_speed_rpm * (float)command_sign;

  if (following_rpm <= -STALL_REVERSE_MIN_RPM)
  {
    if (stall_reverse_elapsed_ms < STALL_REVERSE_CONFIRM_MS)
    {
      stall_reverse_elapsed_ms++;
    }
  }
  else
  {
    stall_reverse_elapsed_ms = 0U;
  }

  if (following_rpm <= feedback_limit_rpm)
  {
    if (stall_elapsed_ms < STALL_CONFIRM_MS)
    {
      stall_elapsed_ms++;
    }
    if ((stall_elapsed_ms >= STALL_CONFIRM_MS) ||
        (stall_reverse_elapsed_ms >= STALL_REVERSE_CONFIRM_MS))
    {
      stall_fault = 1U;
      Step42Control_StopMotion();
      Step42Control_SetUserEnable(0U);
    }
  }
  else
  {
    stall_elapsed_ms = 0U;
  }
}

uint8_t Step42Control_TimerTick1kHz(void)
{
  uint16_t raw_angle = 0U;
  float dt = 1.0f / (float)control_loop_hz;

  mt6701_status = MT6701_ReadRawAngleNoCrc(&raw_angle);
  if (mt6701_status != MT6701_OK)
  {
    return 0U;
  }

  mt6701_raw_angle = raw_angle;
  mt6701_angle_deg = Normalize360(PID_RawToDeg(raw_angle) - zero_offset_deg);
  {
    float unwrapped_angle = MT6701_UpdateUnwrappedDeg(raw_angle);

    /* Speed and tracking modes do not need an ever-growing angle coordinate. */
    if ((control_mode != CTRL_MODE_CLOSED_POSITION) &&
        (control_mode != CTRL_MODE_EXTERNAL_POSITION) &&
        (AbsF(unwrapped_angle) >= ENCODER_UNWRAP_REBASE_DEG))
    {
      Encoder_RebaseFromTimer();
      unwrapped_angle = mt6701_unwrapped_deg;
    }
    if (speed_feedback_valid == 0U)
    {
      speed_last_unwrapped_deg = unwrapped_angle;
      speed_feedback_valid = 1U;
      closed_speed_feedback_steps = 0.0f;
    }
    else
    {
      float delta_deg = unwrapped_angle - speed_last_unwrapped_deg;

      closed_speed_feedback_steps = delta_deg * motor_steps_per_rev / 360.0f / dt;
      speed_last_unwrapped_deg = unwrapped_angle;
      speed_accum_delta_deg += delta_deg;
      speed_accum_time_s += dt;
      if (speed_accum_time_s >= SPEED_ESTIMATE_WINDOW_S)
      {
        float instant_rpm = (speed_accum_delta_deg / 360.0f) * 60.0f / speed_accum_time_s;
        motor_speed_rpm += SPEED_FILTER_ALPHA * (instant_rpm - motor_speed_rpm);
        speed_accum_delta_deg = 0.0f;
        speed_accum_time_s = 0.0f;
      }
    }
    if (control_mode == CTRL_MODE_CLOSED_POSITION)
    {
      WeakPID_Update(unwrapped_angle);
    }
    else if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
    {
      float external_angle = ExternalPosition_UpdateCurrent(raw_angle);

      if ((uint32_t)(HAL_GetTick() - external_position_last_target_ms) >
          external_position_config.timeout_ms)
      {
        /* A lost external controller returns the pipe to its saved center. */
        external_position_target_deg = 0.0f;
      }
      weak_pid_target_deg = external_position_target_deg;
      WeakPID_Update(external_angle);
    }
    else if (control_mode == CTRL_MODE_TRACK_PID)
    {
      float tracking_output = 0.0f;
      if (Step42Tracking_Update(dt, &tracking_output) != 0U)
      {
        Tracking_ApplySpeed(tracking_output);
      }
    }
  }

  StallProtection_Update();

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

float Step42Control_GetMultiTurnAngleDeg(void)
{
  if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    return external_position_current_deg;
  }
  return mt6701_unwrapped_deg;
}

float Step42Control_GetPidOutputSteps(void)
{
  return weak_pid_output_steps;
}

float Step42Control_GetPositionTargetDeg(void)
{
  if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    return external_position_target_deg;
  }
  return weak_pid_target_deg;
}

float Step42Control_GetPositionErrorDeg(void)
{
  if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    return external_position_target_deg - external_position_current_deg;
  }
  if ((control_mode != CTRL_MODE_CLOSED_POSITION) || (weak_pid_enable == 0U))
  {
    return 0.0f;
  }
  return weak_pid_target_deg - mt6701_unwrapped_deg;
}

uint8_t Step42Control_IsPositionPidActive(void)
{
  return (((control_mode == CTRL_MODE_CLOSED_POSITION) ||
           (control_mode == CTRL_MODE_EXTERNAL_POSITION)) &&
          (weak_pid_enable != 0U)) ? 1U : 0U;
}

float Step42Control_GetClosedAccel(void)
{
  return closed_position_accel;
}

uint8_t Step42Control_GetMotionProfile(void)
{
  return (uint8_t)Stepper_GetMotionProfile(&Motor_42);
}

uint8_t Step42Control_SetCurrent(uint8_t hold_current, uint8_t run_current)
{
  uint8_t restore_output = ((motor_user_enabled != 0U) &&
                            (thermal_lockout == 0U) &&
                            (stall_fault == 0U)) ? 1U : 0U;
  uint8_t status;

  if (restore_output != 0U)
  {
    Motor_DisableOutput();
    HAL_Delay(3U);
  }

  status = Tmc2209_SetCurrent(hold_current, run_current);

  if (restore_output != 0U)
  {
    Motor_EnableOutput();
  }
  return status;
}

uint8_t Step42Control_GetHoldCurrent(void)
{
  return Tmc2209_GetHoldCurrent();
}

uint8_t Step42Control_GetRunCurrent(void)
{
  return Tmc2209_GetRunCurrent();
}

float Step42Control_GetSpeedRpm(void)
{
  return motor_speed_rpm;
}

uint8_t Step42Control_SetTrackingBasic(uint8_t mode,
                                       uint8_t speed_percent,
                                       uint8_t accel_percent,
                                       uint8_t frequency_khz)
{
  if (Step42Tracking_SetBasic(mode, speed_percent, accel_percent, frequency_khz) == 0U)
  {
    return 0U;
  }
  if (control_mode == CTRL_MODE_TRACK_PID)
  {
    SetControlLoopFrequency(frequency_khz);
  }
  return 1U;
}

uint8_t Step42Control_SetTrackingPid(uint8_t loop, float kp, float ki, float kd)
{
  return Step42Tracking_SetPid(loop, kp, ki, kd);
}

uint8_t Step42Control_SetTrackingAdvanced(float filter_alpha,
                                          float gain_min,
                                          float gain_max,
                                          float full_error,
                                          float prediction_ms,
                                          uint16_t timeout_ms,
                                          float single_deadband,
                                          float outer_deadband,
                                          float inner_deadband)
{
  return Step42Tracking_SetAdvanced(filter_alpha,
                                    gain_min,
                                    gain_max,
                                    full_error,
                                    prediction_ms,
                                    timeout_ms,
                                    single_deadband,
                                    outer_deadband,
                                    inner_deadband);
}

void Step42Control_SetTrackingInput(float primary_error, float secondary_feedback)
{
  if (control_mode != CTRL_MODE_TRACK_PID)
  {
    return;
  }
  if (motor_user_enabled == 0U)
  {
    Step42Control_SetUserEnable(1U);
  }
  if (tracking_error_reverse != 0U)
  {
    primary_error = -primary_error;
  }
  Step42Tracking_SetInput(primary_error, secondary_feedback);
}

void Step42Control_SetProgramSignals(uint8_t target_detected,
                                     uint8_t custom1,
                                     uint8_t custom2)
{
  program_target_detected = (target_detected != 0U) ? 1U : 0U;
  program_custom_signal1 = (custom1 != 0U) ? 1U : 0U;
  program_custom_signal2 = (custom2 != 0U) ? 1U : 0U;
}

void Step42Control_SetTrackingDirection(uint8_t reverse)
{
  tracking_error_reverse = (reverse != 0U) ? 1U : 0U;
}

uint8_t Step42Control_GetTrackingDirection(void)
{
  return tracking_error_reverse;
}

void Step42Control_ResetTrackingDefaults(void)
{
  Step42Tracking_ResetDefaults();
  if (control_mode == CTRL_MODE_TRACK_PID)
  {
    SetControlLoopFrequency(Step42Tracking_GetConfig()->frequency_khz);
  }
}

uint8_t Step42Control_SetExternalPositionConfig(const Step42ExternalPositionConfig_t *config)
{
  if ((config == 0) || (config->accel == 0U) ||
      (config->timeout_ms < 20U) || (config->timeout_ms > 5000U) ||
      (config->minimum_deg < -7200.0f) || (config->maximum_deg > 7200.0f) ||
      (config->minimum_deg >= config->maximum_deg) ||
      (config->kp < 0.0f) || (config->kp > 5000.0f) ||
      (config->ki < 0.0f) || (config->ki > 5000.0f) ||
      (config->kd < 0.0f) || (config->kd > 5000.0f) ||
      (config->output_limit < 1.0f) ||
      (config->output_limit > (float)Step42Control_GetPulseSpeedLimitForMicrosteps(config->microsteps)) ||
      (config->deadband < 0.0f) || (config->deadband > 65.535f) ||
      (config->center_raw_deg < 0.0f) || (config->center_raw_deg >= 360.0f) ||
      (config->center_valid > 1U))
  {
    return 0U;
  }

  switch (config->microsteps)
  {
  case 1U:
  case 2U:
  case 4U:
  case 8U:
  case 16U:
  case 32U:
  case 64U:
  case 128U:
  case 256U:
    break;
  default:
    return 0U;
  }

  if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    if ((motor_microsteps != config->microsteps) &&
        (Tmc2209_SetMicrosteps(config->microsteps) == 0U))
    {
      return 0U;
    }
    motor_microsteps = config->microsteps;
    motor_steps_per_rev = MOTOR_BASE_STEPS_PER_REV * (float)motor_microsteps;
  }

  external_position_config = *config;
  external_position_config.center_raw_deg = Normalize360(config->center_raw_deg);
  if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    ExternalPosition_ApplyConfig();
    external_position_target_deg = ClampF(external_position_target_deg,
                                          external_position_config.minimum_deg,
                                          external_position_config.maximum_deg);
  }
  return 1U;
}

const Step42ExternalPositionConfig_t *Step42Control_GetExternalPositionConfig(void)
{
  return &external_position_config;
}

uint8_t Step42Control_SetExternalPositionCenter(void)
{
  if (mt6701_status != MT6701_OK)
  {
    return 0U;
  }

  external_position_config.center_raw_deg = PID_RawToDeg(mt6701_raw_angle);
  external_position_config.center_valid = 1U;
  ExternalPosition_Rebase();
  external_position_target_deg = 0.0f;
  external_position_last_target_ms = HAL_GetTick();
  if (control_mode == CTRL_MODE_EXTERNAL_POSITION)
  {
    weak_pid_target_deg = 0.0f;
    PID_Reset(&weak_pid);
  }
  return 1U;
}

uint8_t Step42Control_SetExternalPositionTarget(float target_deg)
{
  if ((control_mode != CTRL_MODE_EXTERNAL_POSITION) ||
      (external_position_config.center_valid == 0U) ||
      (target_deg < external_position_config.minimum_deg) ||
      (target_deg > external_position_config.maximum_deg))
  {
    return 0U;
  }

  if (motor_user_enabled == 0U)
  {
    Step42Control_SetUserEnable(1U);
  }
  external_position_target_deg = target_deg;
  external_position_last_target_ms = HAL_GetTick();
  weak_pid_enable = 1U;
  return 1U;
}

float Step42Control_GetExternalPositionTargetDeg(void)
{
  return external_position_target_deg;
}

float Step42Control_GetExternalPositionCurrentDeg(void)
{
  return external_position_current_deg;
}

void Step42Control_SetThermalLockout(uint8_t lockout)
{
  uint8_t new_lockout = (lockout != 0U) ? 1U : 0U;

  if (new_lockout == thermal_lockout)
  {
    return;
  }
  thermal_lockout = new_lockout;
  if (thermal_lockout != 0U)
  {
    Step42Control_SetUserEnable(0U);
  }
}

uint8_t Step42Control_IsThermalLocked(void)
{
  return thermal_lockout;
}

const Step42TrackingConfig_t *Step42Control_GetTrackingConfig(void)
{
  return Step42Tracking_GetConfig();
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
