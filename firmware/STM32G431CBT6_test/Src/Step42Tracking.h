#ifndef STEP42_TRACKING_H
#define STEP42_TRACKING_H

#include <stdint.h>

#define STEP42_TRACK_OFF (0U)
#define STEP42_TRACK_SINGLE (1U)
#define STEP42_TRACK_CASCADE (2U)

#define STEP42_TRACK_LOOP_SINGLE (0U)
#define STEP42_TRACK_LOOP_OUTER (1U)
#define STEP42_TRACK_LOOP_INNER (2U)

typedef struct
{
    float kp;
    float ki;
    float kd;
    float deadband;
    float integral_limit;
} Step42TrackingPidConfig_t;

typedef struct
{
    uint8_t mode;
    uint8_t speed_percent;
    uint8_t accel_percent;
    uint8_t frequency_khz;
    Step42TrackingPidConfig_t single;
    Step42TrackingPidConfig_t outer;
    Step42TrackingPidConfig_t inner;
    float filter_alpha;
    float dynamic_gain_min;
    float dynamic_gain_max;
    float dynamic_full_error;
    float prediction_ms;
    uint16_t timeout_ms;
} Step42TrackingConfig_t;

void Step42Tracking_Init(void);
void Step42Tracking_ResetDefaults(void);
uint8_t Step42Tracking_SetBasic(uint8_t mode,
                                uint8_t speed_percent,
                                uint8_t accel_percent,
                                uint8_t frequency_khz);
uint8_t Step42Tracking_SetPid(uint8_t loop,
                              float kp,
                              float ki,
                              float kd);
uint8_t Step42Tracking_SetAdvanced(float filter_alpha,
                                   float gain_min,
                                   float gain_max,
                                   float full_error,
                                   float prediction_ms,
                                   uint16_t timeout_ms,
                                   float single_deadband,
                                   float outer_deadband,
                                   float inner_deadband);
void Step42Tracking_SetInput(float primary_error, float secondary_feedback);
uint8_t Step42Tracking_Update(float dt, float *speed_output);
void Step42Tracking_Stop(void);
const Step42TrackingConfig_t *Step42Tracking_GetConfig(void);
float Step42Tracking_GetPrimaryError(void);
float Step42Tracking_GetSecondaryFeedback(void);
float Step42Tracking_GetOutput(void);
float Step42Tracking_GetMaxSpeed(void);
float Step42Tracking_GetAccel(void);

#endif
