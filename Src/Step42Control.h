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

#define STEP42_PROGRAM_MAX_STEPS (8U)

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

#define STEP42_PROGRAM_SPEED_FLAG_RPM  (0x01U)

typedef struct
{
  uint8_t op;
  uint8_t flags;
  uint16_t speed;
  uint16_t accel;
  uint16_t duration_ms;
  int32_t value;
} Step42ProgramStep_t;

void Step42Control_Init(void);
void Step42Control_SetUserEnable(uint8_t enable);
uint8_t Step42Control_SetMode(uint8_t mode);
void Step42Control_MoveToAngleByMode(float target_angle);
uint8_t Step42Control_MoveRelativeAngleByMode(float delta_angle);
uint8_t Step42Control_MoveStepsByMode(int32_t steps, float speed, float accel);
void Step42Control_RunSpeedByMode(float speed, float accel);
void Step42Control_StopMotion(void);
void Step42Control_AbortProgram(void);
uint8_t Step42Control_SetMicrosteps(uint16_t microsteps);
uint8_t Step42Control_SetClosedAccel(float accel);
void Step42Control_SetPositionPID(float kp, float ki, float kd, float output_limit, float deadband);
void Step42Control_GetPositionPID(float *kp, float *ki, float *kd, float *output_limit, float *deadband);
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
void Step42Control_ResetTrackingDefaults(void);
void Step42Control_SetThermalLockout(uint8_t lockout);

uint8_t Step42Control_IsUserEnabled(void);
uint8_t Step42Control_IsMotionActive(void);
uint8_t Step42Control_IsStallFault(void);
uint8_t Step42Control_GetMode(void);
uint16_t Step42Control_GetMicrosteps(void);
float Step42Control_GetClosedAccel(void);
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
