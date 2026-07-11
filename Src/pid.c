#include "pid.h"

#define PID_MT6701_RESOLUTION 16384.0f

static float PID_Clamp(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }

    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float dt,
              float output_limit,
              float integral_limit,
              float deadband)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->deadband = deadband;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PID_t *pid, float target, float feedback)
{
    float error;
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    error = PID_AngleErrorDeg(target, feedback);
    if ((error < pid->deadband) && (error > -pid->deadband))
    {
        pid->prev_error = error;
        return 0.0f;
    }

    pid->integral += error * pid->dt;
    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);

    derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return PID_Clamp(output, pid->output_limit);
}

float PID_UpdateLinear(PID_t *pid, float target, float feedback)
{
    return PID_UpdateError(pid, target - feedback);
}

float PID_UpdateError(PID_t *pid, float error)
{
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    if ((error < pid->deadband) && (error > -pid->deadband))
    {
        pid->prev_error = error;
        return 0.0f;
    }

    pid->integral += error * pid->dt;
    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);

    derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return PID_Clamp(output, pid->output_limit);
}

float PID_AngleErrorDeg(float target, float feedback)
{
    float error = target - feedback;

    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

float PID_RawToDeg(uint16_t raw_angle)
{
    return ((float)(raw_angle & 0x3FFFU)) * 360.0f / PID_MT6701_RESOLUTION;
}
