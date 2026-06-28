#ifndef STEP42_CONTROL_H
#define STEP42_CONTROL_H

#include "main.h"
#include "Stepper.h"
#include "MT6701.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern Stepper_t Motor_42;

void Step42Control_Init(void);
void Step42Control_SetUserEnable(uint8_t enable);
void Step42Control_SetMode(uint8_t mode);
void Step42Control_MoveToAngleByMode(float target_angle);
uint8_t Step42Control_MoveStepsByMode(int32_t steps, float speed, float accel);
void Step42Control_RunSpeedByMode(float speed, float accel);
void Step42Control_StopMotion(void);
uint8_t Step42Control_SetMicrosteps(uint16_t microsteps);
uint8_t Step42Control_SetClosedAccel(float accel);
void Step42Control_SetPositionPID(float kp, float ki, float kd, float output_limit, float deadband);
void Step42Control_GetPositionPID(float *kp, float *ki, float *kd, float *output_limit, float *deadband);
uint8_t Step42Control_SetHomeMode(uint8_t mode);
uint8_t Step42Control_SetCurrentZero(void);
uint8_t Step42Control_Home(uint8_t mode);
void Step42Control_HandleKey(int8_t key_flag);
uint8_t Step42Control_TimerTick1kHz(void);

uint8_t Step42Control_IsUserEnabled(void);
uint8_t Step42Control_GetMode(void);
uint16_t Step42Control_GetMicrosteps(void);
float Step42Control_GetClosedAccel(void);
uint8_t Step42Control_GetHomeMode(void);
uint8_t Step42Control_IsZeroValid(void);
uint16_t Step42Control_GetRawAngle(void);
float Step42Control_GetAngleDeg(void);
float Step42Control_GetPidOutputSteps(void);
MT6701_Status_t Step42Control_GetEncoderStatus(void);

#ifdef __cplusplus
}
#endif

#endif
