#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float dt;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    float deadband;
} PID_t;

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float dt,
              float output_limit,
              float integral_limit,
              float deadband);

void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float target, float feedback);
float PID_UpdateLinear(PID_t *pid, float target, float feedback);
float PID_AngleErrorDeg(float target, float feedback);
float PID_RawToDeg(uint16_t raw_angle);

#endif
