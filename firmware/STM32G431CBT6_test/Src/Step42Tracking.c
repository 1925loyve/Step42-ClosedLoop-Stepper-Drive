#include "Step42Tracking.h"
#include "main.h"
#include "pid.h"

#define TRACK_DEFAULT_SPEED_PERCENT (100U)
#define TRACK_DEFAULT_ACCEL_PERCENT (100U)
#define TRACK_DEFAULT_FREQUENCY_KHZ (1U)
#define TRACK_MAX_SPEED_PPS (15000.0f)
#define TRACK_MAX_ACCEL_PPS2 (30000.0f)

static Step42TrackingConfig_t config;
static PID_t single_pid;
static PID_t outer_pid;
static PID_t inner_pid;
static float raw_primary_error = 0.0f;
static float secondary_feedback = 0.0f;
static float filtered_error = 0.0f;
static float previous_filtered_error = 0.0f;
static float output_speed = 0.0f;
static uint32_t last_input_ms = 0U;
static uint8_t filter_valid = 0U;
static uint8_t input_valid = 0U;

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

static float AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float Step42Tracking_GetMaxSpeed(void)
{
    return (float)config.speed_percent * (TRACK_MAX_SPEED_PPS / 100.0f);
}

float Step42Tracking_GetAccel(void)
{
    return (float)config.accel_percent * (TRACK_MAX_ACCEL_PPS2 / 100.0f);
}

static void ApplyPidConfig(PID_t *pid,
                           const Step42TrackingPidConfig_t *pid_config,
                           float dt,
                           float output_limit)
{
    PID_Init(pid,
             pid_config->kp,
             pid_config->ki,
             pid_config->kd,
             dt,
             output_limit,
             pid_config->integral_limit,
             pid_config->deadband);
}

static void SyncPids(float dt)
{
    float max_speed = Step42Tracking_GetMaxSpeed();

    ApplyPidConfig(&single_pid, &config.single, dt, max_speed);
    ApplyPidConfig(&outer_pid, &config.outer, dt, max_speed);
    ApplyPidConfig(&inner_pid, &config.inner, dt, max_speed);
}

void Step42Tracking_ResetDefaults(void)
{
    config.mode = STEP42_TRACK_SINGLE;
    config.speed_percent = TRACK_DEFAULT_SPEED_PERCENT;
    config.accel_percent = TRACK_DEFAULT_ACCEL_PERCENT;
    config.frequency_khz = TRACK_DEFAULT_FREQUENCY_KHZ;
    config.advanced_flags = 0U;
    config.reserved[0] = 0U;
    config.reserved[1] = 0U;
    config.reserved[2] = 0U;

    config.single.kp = 180.0f;
    config.single.ki = 0.2f;
    config.single.kd = 0.0f;
    config.single.deadband = 0.08f;
    config.single.integral_limit = 20.0f;

    config.outer.kp = 1.0f;
    config.outer.ki = 0.0f;
    config.outer.kd = 0.0f;
    config.outer.deadband = 0.08f;
    config.outer.integral_limit = 20.0f;

    config.inner.kp = 180.0f;
    config.inner.ki = 0.2f;
    config.inner.kd = 0.0f;
    config.inner.deadband = 0.08f;
    config.inner.integral_limit = 20.0f;

    config.filter_alpha = 0.25f;
    config.dynamic_gain_min = 1.0f;
    config.dynamic_gain_max = 1.0f;
    config.dynamic_full_error = 10.0f;
    config.prediction_ms = 0.0f;
    config.timeout_ms = 250U;
    SyncPids(0.001f);
    Step42Tracking_Stop();
}

void Step42Tracking_Init(void)
{
    Step42Tracking_ResetDefaults();
}

uint8_t Step42Tracking_SetBasic(uint8_t mode,
                                uint8_t speed_percent,
                                uint8_t accel_percent,
                                uint8_t frequency_khz)
{
    if ((mode < STEP42_TRACK_SINGLE) || (mode > STEP42_TRACK_CASCADE) ||
        (speed_percent < 1U) || (speed_percent > 100U) ||
        (accel_percent < 1U) || (accel_percent > 100U) ||
        (frequency_khz < 1U) || (frequency_khz > 10U))
    {
        return 0U;
    }

    config.mode = mode;
    config.speed_percent = speed_percent;
    config.accel_percent = accel_percent;
    config.frequency_khz = frequency_khz;
    SyncPids(1.0f / ((float)frequency_khz * 1000.0f));
    Step42Tracking_Stop();
    return 1U;
}

uint8_t Step42Tracking_SetPid(uint8_t loop, float kp, float ki, float kd)
{
    Step42TrackingPidConfig_t *target;

    if ((kp < 0.0f) || (kp > 5000.0f) ||
        (ki < 0.0f) || (ki > 5000.0f) ||
        (kd < 0.0f) || (kd > 5000.0f))
    {
        return 0U;
    }

    if (loop == STEP42_TRACK_LOOP_SINGLE)
    {
        target = &config.single;
    }
    else if (loop == STEP42_TRACK_LOOP_OUTER)
    {
        target = &config.outer;
    }
    else if (loop == STEP42_TRACK_LOOP_INNER)
    {
        target = &config.inner;
    }
    else
    {
        return 0U;
    }

    target->kp = kp;
    target->ki = ki;
    target->kd = kd;
    SyncPids(1.0f / ((float)config.frequency_khz * 1000.0f));
    return 1U;
}

uint8_t Step42Tracking_SetAdvanced(float filter_alpha,
                                   float gain_min,
                                   float gain_max,
                                   float full_error,
                                   float prediction_ms,
                                   uint16_t timeout_ms,
                                   float single_deadband,
                                   float outer_deadband,
                                   float inner_deadband,
                                   uint8_t advanced_flags)
{
    if ((filter_alpha <= 0.0f) || (filter_alpha > 1.0f) ||
        (gain_min < 0.1f) || (gain_min > 10.0f) ||
        (gain_max < gain_min) || (gain_max > 10.0f) ||
        (full_error <= 0.0f) || (full_error > 655.35f) ||
        (prediction_ms < 0.0f) || (prediction_ms > 200.0f) ||
        (timeout_ms < 20U) || (timeout_ms > 5000U) ||
        (single_deadband < 0.0f) || (single_deadband > 65.535f) ||
        (outer_deadband < 0.0f) || (outer_deadband > 65.535f) ||
        (inner_deadband < 0.0f) || (inner_deadband > 65.535f) ||
        ((advanced_flags & (uint8_t)~STEP42_TRACK_ADV_ALL) != 0U))
    {
        return 0U;
    }

    config.filter_alpha = filter_alpha;
    config.dynamic_gain_min = gain_min;
    config.dynamic_gain_max = gain_max;
    config.dynamic_full_error = full_error;
    config.prediction_ms = prediction_ms;
    config.timeout_ms = timeout_ms;
    config.single.deadband = single_deadband;
    config.outer.deadband = outer_deadband;
    config.inner.deadband = inner_deadband;
    config.advanced_flags = advanced_flags;
    SyncPids(1.0f / ((float)config.frequency_khz * 1000.0f));
    return 1U;
}

void Step42Tracking_SetInput(float primary_error, float secondary)
{
    raw_primary_error = primary_error;
    secondary_feedback = secondary;
    last_input_ms = HAL_GetTick();
    input_valid = 1U;
}

uint8_t Step42Tracking_Update(float dt, float *speed_output)
{
    float derivative;
    float predicted_error;
    float gain_ratio;
    float gain;
    float command;

    if ((speed_output == 0) || (config.mode == STEP42_TRACK_OFF))
    {
        return 0U;
    }
    if (input_valid == 0U)
    {
        *speed_output = 0.0f;
        return 1U;
    }
    if ((uint32_t)(HAL_GetTick() - last_input_ms) > config.timeout_ms)
    {
        Step42Tracking_Stop();
        *speed_output = 0.0f;
        return 1U;
    }

    if ((config.advanced_flags & STEP42_TRACK_ADV_FILTER) == 0U)
    {
        previous_filtered_error = (filter_valid != 0U) ?
                                  filtered_error : raw_primary_error;
        filtered_error = raw_primary_error;
        filter_valid = 1U;
    }
    else if (filter_valid == 0U)
    {
        filtered_error = raw_primary_error;
        previous_filtered_error = filtered_error;
        filter_valid = 1U;
    }
    else
    {
        previous_filtered_error = filtered_error;
        filtered_error += config.filter_alpha * (raw_primary_error - filtered_error);
    }

    derivative = (dt > 0.0f) ? ((filtered_error - previous_filtered_error) / dt) : 0.0f;
    predicted_error = filtered_error;
    if ((config.advanced_flags & STEP42_TRACK_ADV_PREDICTION) != 0U)
    {
        predicted_error += derivative * config.prediction_ms * 0.001f;
    }
    gain = 1.0f;
    if ((config.advanced_flags & STEP42_TRACK_ADV_GAIN) != 0U)
    {
        gain_ratio = ClampF(AbsF(predicted_error) / config.dynamic_full_error, 0.0f, 1.0f);
        gain = config.dynamic_gain_min +
               (config.dynamic_gain_max - config.dynamic_gain_min) * gain_ratio;
    }

    single_pid.dt = dt;
    outer_pid.dt = dt;
    inner_pid.dt = dt;
    if (config.mode == STEP42_TRACK_CASCADE)
    {
        float inner_target = PID_UpdateError(&outer_pid, predicted_error);
        command = PID_UpdateError(&inner_pid, inner_target - secondary_feedback);
    }
    else
    {
        command = PID_UpdateError(&single_pid, predicted_error);
    }

    output_speed = ClampF(command * gain,
                          -Step42Tracking_GetMaxSpeed(),
                          Step42Tracking_GetMaxSpeed());
    *speed_output = output_speed;
    return 1U;
}

void Step42Tracking_Stop(void)
{
    PID_Reset(&single_pid);
    PID_Reset(&outer_pid);
    PID_Reset(&inner_pid);
    filter_valid = 0U;
    input_valid = 0U;
    last_input_ms = 0U;
    output_speed = 0.0f;
}

const Step42TrackingConfig_t *Step42Tracking_GetConfig(void)
{
    return &config;
}

float Step42Tracking_GetPrimaryError(void)
{
    return raw_primary_error;
}

float Step42Tracking_GetSecondaryFeedback(void)
{
    return secondary_feedback;
}

float Step42Tracking_GetOutput(void)
{
    return output_speed;
}
