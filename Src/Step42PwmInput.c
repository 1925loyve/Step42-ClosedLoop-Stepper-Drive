#include "Step42PwmInput.h"
#include "Step42BoardMode.h"
#include "Step42Control.h"
#include "Step42SlaveProtocol.h"

#define PWM_MIN_PERIOD_US (2500UL)
#define PWM_MAX_PERIOD_US (25000UL)
#define PWM_MIN_PULSE_US (900UL)
#define PWM_MAX_PULSE_US (2100UL)
#define PWM_COMMAND_MIN_US (1000.0f)
#define PWM_COMMAND_MAX_US (2000.0f)
#define PWM_SIGNAL_TIMEOUT_MS (100UL)
#define PWM_ANGLE_CHANGE_MIN_DEG (0.05f)

extern TIM_HandleTypeDef htim2;

static volatile uint32_t captured_period_us = 0U;
static volatile uint32_t captured_pulse_us = 0U;
static volatile uint8_t capture_pending = 0U;
static uint32_t last_valid_ms = 0U;
static float last_angle_deg = -1.0f;

static float AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void Step42PwmInput_Init(void)
{
    if (Step42BoardMode_Get() != STEP42_BOARD_PWM)
    {
        return;
    }

    Step42Control_SetMode(CTRL_MODE_CLOSED_POSITION);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

void Step42PwmInput_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance != TIM2) ||
        (Step42BoardMode_Get() != STEP42_BOARD_PWM) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1))
    {
        return;
    }

    captured_period_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    captured_pulse_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    capture_pending = 1U;
}

void Step42PwmInput_Process(void)
{
    uint32_t period;
    uint32_t pulse;
    float angle;

    if (Step42BoardMode_Get() != STEP42_BOARD_PWM)
    {
        return;
    }

    if (capture_pending != 0U)
    {
        __disable_irq();
        period = captured_period_us;
        pulse = captured_pulse_us;
        capture_pending = 0U;
        __enable_irq();

        if ((period >= PWM_MIN_PERIOD_US) && (period <= PWM_MAX_PERIOD_US) &&
            (pulse >= PWM_MIN_PULSE_US) && (pulse <= PWM_MAX_PULSE_US))
        {
            float limited_pulse = (float)pulse;
            if (limited_pulse < PWM_COMMAND_MIN_US)
            {
                limited_pulse = PWM_COMMAND_MIN_US;
            }
            else if (limited_pulse > PWM_COMMAND_MAX_US)
            {
                limited_pulse = PWM_COMMAND_MAX_US;
            }
            angle = (limited_pulse - PWM_COMMAND_MIN_US) * 360.0f /
                    (PWM_COMMAND_MAX_US - PWM_COMMAND_MIN_US);
            last_valid_ms = HAL_GetTick();
            if ((last_angle_deg < 0.0f) ||
                (AbsF(angle - last_angle_deg) >= PWM_ANGLE_CHANGE_MIN_DEG))
            {
                Step42Control_MoveToAngleByMode(angle);
                last_angle_deg = angle;
            }
        }
    }

    if ((last_valid_ms != 0U) &&
        ((uint32_t)(HAL_GetTick() - last_valid_ms) > PWM_SIGNAL_TIMEOUT_MS))
    {
        Step42Control_StopMotion();
        last_valid_ms = 0U;
        last_angle_deg = -1.0f;
    }
}
