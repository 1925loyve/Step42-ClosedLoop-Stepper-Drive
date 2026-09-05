#ifndef STEP42_CONTROL_H
#define STEP42_CONTROL_H

#include "main.h"
#include "Stepper.h"
#include "MT6701.h"
#include "Step42Tracking.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern Stepper_t Motor_42;

#define STEP42_PROGRAM_MAX_STEPS (32U)

#define STEP42_PROGRAM_OP_POSITION_ABS (1U)
#define STEP42_PROGRAM_OP_POSITION_REL (2U)
#define STEP42_PROGRAM_OP_OPEN_STEPS   (3U)
#define STEP42_PROGRAM_OP_SPEED        (4U)
#define STEP42_PROGRAM_OP_DELAY        (5U)
#define STEP42_PROGRAM_OP_HOME         (6U)
#define STEP42_PROGRAM_OP_TRACK        (7U)
#define STEP42_PROGRAM_OP_STOP         (8U)
#define STEP42_PROGRAM_OP_MICROSTEP    (9U)
#define STEP42_PROGRAM_OP_CHOPPER      (10U)
#define STEP42_PROGRAM_OP_EXTERNAL_POSITION (11U)
#define STEP42_PROGRAM_OP_DRIVE_STATE  (12U)
#define STEP42_PROGRAM_OP_GOTO         (13U)
#define STEP42_PROGRAM_OP_BRANCH       (14U)

#define STEP42_PROGRAM_SPEED_FLAG_RPM  (0x01U)
#define STEP42_PROGRAM_STATE_MODE      (0x01U)
#define STEP42_PROGRAM_STATE_MICROSTEP (0x02U)
#define STEP42_PROGRAM_STATE_CHOPPER   (0x04U)
#define STEP42_PROGRAM_STATE_PROFILE   (0x08U)

#define STEP42_PROGRAM_CONDITION_NONE         (0U)
#define STEP42_PROGRAM_CONDITION_SIGNAL_TRUE  (1U)
#define STEP42_PROGRAM_CONDITION_SIGNAL_FALSE (2U)

#define STEP42_PROGRAM_SIGNAL_TARGET  (0U)
#define STEP42_PROGRAM_SIGNAL_CUSTOM1 (1U)
#define STEP42_PROGRAM_SIGNAL_CUSTOM2 (2U)
#define STEP42_PROGRAM_JUMP_NONE      (0xFFU)

typedef struct
{
  uint8_t op;
  uint8_t flags;
  uint8_t condition;
  uint8_t signal;
  uint8_t jump_index;
  uint8_t reserved;
  uint16_t speed;
  uint16_t accel;
  uint16_t duration_ms;
  int32_t value;
} Step42ProgramStep_t;

/* External continuous position targets are relative to this saved software
   center. It makes the feature independent of each MT6701's raw angle. */
typedef struct
{
  uint16_t microsteps;
  uint16_t accel;
  float kp;
  float ki;
  float kd;
  float output_limit;
  float deadband;
  float minimum_deg;
  float maximum_deg;
  uint16_t timeout_ms;
  float center_raw_deg;
  uint8_t center_valid;
} Step42ExternalPositionConfig_t;

void Step42Control_Init(void);
void Step42Control_SetUserEnable(uint8_t enable);
uint8_t Step42Control_SetMode(uint8_t mode);
void Step42Control_MoveToAngleByMode(float target_angle);
void Step42Control_MoveToPwmAngle(float target_angle);
uint8_t Step42Control_MoveRelativeAngleByMode(float delta_angle);
uint8_t Step42Control_MoveStepsByMode(int32_t steps, float speed, float accel);
void Step42Control_RunSpeedByMode(float speed, float accel);
void Step42Control_StopMotion(void);
void Step42Control_AbortProgram(void);
uint8_t Step42Control_SetMicrosteps(uint16_t microsteps);
uint8_t Step42Control_SetClosedAccel(float accel);
uint8_t Step42Control_SetMotionProfile(uint8_t profile);
uint8_t Step42Control_SetCurrent(uint8_t hold_current, uint8_t run_current);
uint8_t Step42Control_GetHoldCurrent(void);
uint8_t Step42Control_GetRunCurrent(void);
void Step42Control_SetPositionPID(float kp, float ki, float kd, float output_limit, float deadband);
void Step42Control_GetPositionPID(float *kp, float *ki, float *kd, float *output_limit, float *deadband);
uint8_t Step42Control_SaveRuntimeConfig(void);
uint8_t Step42Control_SetCanNodeId(uint8_t node_id);
uint8_t Step42Control_GetCanNodeId(void);
uint8_t Step42Control_SetHomeMode(uint8_t mode);
uint8_t Step42Control_SetCurrentZero(void);
uint8_t Step42Control_Home(uint8_t mode);
void Step42Control_HandleKey(int8_t key_flag);
uint8_t Step42Control_SetProgramMode(uint8_t enable);
uint8_t Step42Control_IsProgramMode(void);
uint8_t Step42Control_SetProgramStep(uint8_t index, const Step42ProgramStep_t *step);
uint8_t Step42Control_StageProgram(uint8_t count);
uint8_t Step42Control_SaveProgram(uint8_t count);
uint8_t Step42Control_StartProgram(void);
uint8_t Step42Control_StartOfflineProgram(void);
void Step42Control_ProcessProgram(void);
uint8_t Step42Control_IsProgramRunning(void);
uint8_t Step42Control_GetProgramCount(void);
void Step42Control_MarkExternalTask(void);
uint8_t Step42Control_IsExternalTaskActive(void);
uint8_t Step42Control_TimerTick1kHz(void);
uint8_t Step42Control_SetTrackingBasic(uint8_t mode,
                                       uint8_t speed_percent,
                                       uint8_t accel_percent,
                                       uint8_t frequency_khz);
uint8_t Step42Control_SetTrackingPid(uint8_t loop, float kp, float ki, float kd);
uint8_t Step42Control_SetTrackingAdvanced(float filter_alpha,
                                          float gain_min,
                                          float gain_max,
                                          float full_error,
                                          float prediction_ms,
                                          uint16_t timeout_ms,
                                          float single_deadband,
                                          float outer_deadband,
                                          float inner_deadband);
void Step42Control_SetTrackingInput(float primary_error, float secondary_feedback);
void Step42Control_SetProgramSignals(uint8_t target_detected,
                                     uint8_t custom1,
                                     uint8_t custom2);
void Step42Control_SetTrackingDirection(uint8_t reverse);
uint8_t Step42Control_GetTrackingDirection(void);
void Step42Control_ResetTrackingDefaults(void);
uint8_t Step42Control_SetExternalPositionConfig(const Step42ExternalPositionConfig_t *config);
const Step42ExternalPositionConfig_t *Step42Control_GetExternalPositionConfig(void);
uint8_t Step42Control_SetExternalPositionCenter(void);
uint8_t Step42Control_SetExternalPositionTarget(float target_deg);
float Step42Control_GetExternalPositionTargetDeg(void);
float Step42Control_GetExternalPositionCurrentDeg(void);
void Step42Control_SetThermalLockout(uint8_t lockout);

uint8_t Step42Control_IsUserEnabled(void);
uint8_t Step42Control_IsMotionActive(void);
uint8_t Step42Control_IsStallFault(void);
uint8_t Step42Control_GetMode(void);
uint16_t Step42Control_GetMicrosteps(void);
float Step42Control_GetClosedAccel(void);
uint8_t Step42Control_GetMotionProfile(void);
uint8_t Step42Control_GetHomeMode(void);
uint8_t Step42Control_IsZeroValid(void);
uint16_t Step42Control_GetRawAngle(void);
float Step42Control_GetAngleDeg(void);
float Step42Control_GetMultiTurnAngleDeg(void);
float Step42Control_GetPidOutputSteps(void);
float Step42Control_GetPositionTargetDeg(void);
float Step42Control_GetPositionErrorDeg(void);
uint8_t Step42Control_IsPositionPidActive(void);
float Step42Control_GetSpeedRpm(void);
uint8_t Step42Control_IsThermalLocked(void);
const Step42TrackingConfig_t *Step42Control_GetTrackingConfig(void);
MT6701_Status_t Step42Control_GetEncoderStatus(void);

#ifdef __cplusplus
}
#endif

#endif
