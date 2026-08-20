#include "stepper.h"

#define STEPPER_SCURVE_RAMP_TIME_S (0.030f)
#define STEPPER_MIN_SPEED          (1.0f)

static float Stepper_AbsF(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static float Stepper_SignF(float value)
{
	return (value >= 0.0f) ? 1.0f : -1.0f;
}

static uint32_t Stepper_SpeedToDelay(const Stepper_t *stepper, float speed)
{
	uint32_t delay = (uint32_t)(stepper->timer_freq / Stepper_AbsF(speed));

	if (delay < 100U)
	{
		delay = 100U;
	}
	else if (delay > 600000U)
	{
		delay = 600000U;
	}
	return delay;
}

/*
 * S-curve speed planning is deliberately executed by TIM1, the owner of the
 * STEP period. TIM6 may update target_speed through PID, but never writes TIM1
 * timing registers or step_delay.
 */
static uint8_t Stepper_UpdateSCurveAtPulse(Stepper_t *stepper, uint32_t *new_delay)
{
	float target_speed = stepper->target_speed;
	float delta_speed;
	float desired_accel;
	float accel_step;
	float braking_speed;
	float next_speed;
	float dt;

	if (Stepper_AbsF(target_speed) < STEPPER_MIN_SPEED)
	{
		Stepper_Stop(stepper);
		return 0U;
	}

	dt = (float)stepper->step_delay / stepper->timer_freq;
	delta_speed = target_speed - stepper->current_speed;
	if (Stepper_AbsF(delta_speed) < 0.001f)
	{
		desired_accel = 0.0f;
	}
	else
	{
		braking_speed = (stepper->current_accel * stepper->current_accel) /
		                 (2.0f * stepper->jerk);
		if (((stepper->current_accel * delta_speed) > 0.0f) &&
			(Stepper_AbsF(delta_speed) <= braking_speed))
		{
			desired_accel = 0.0f;
		}
		else
		{
			desired_accel = Stepper_SignF(delta_speed) * stepper->accel;
		}
	}

	accel_step = stepper->jerk * dt;
	if ((desired_accel - stepper->current_accel) > accel_step)
	{
		stepper->current_accel += accel_step;
	}
	else if ((stepper->current_accel - desired_accel) > accel_step)
	{
		stepper->current_accel -= accel_step;
	}
	else
	{
		stepper->current_accel = desired_accel;
	}

	next_speed = stepper->current_speed + stepper->current_accel * dt;
	if (((delta_speed > 0.0f) && (next_speed >= target_speed)) ||
		((delta_speed < 0.0f) && (next_speed <= target_speed)))
	{
		next_speed = target_speed;
		stepper->current_accel = 0.0f;
	}

	stepper->current_speed = next_speed;
	stepper->state = (Stepper_AbsF(target_speed - next_speed) < 0.001f) ? RUN : ACCEL;
	*new_delay = Stepper_SpeedToDelay(stepper, next_speed);
	return 1U;
}

// 1. 初始化函数
void Stepper_Init(Stepper_t *stepper, TIM_HandleTypeDef *htim, uint32_t tim_channel,
				  GPIO_TypeDef *dir_port, uint16_t dir_pin,Stepper_Mode_t mode)
{
	stepper->htim = htim;
	stepper->tim_channel = tim_channel;
	stepper->dir_port = dir_port;
	stepper->dir_pin = dir_pin;
	stepper->run_mode = mode;

	stepper->timer_freq = 1000000.0f;

	stepper->state = STOP;
	stepper->profile = STEPPER_PROFILE_TRAPEZOID;
	stepper->current_pos = 0;

	stepper->target_speed = 0.0f;
	stepper->current_speed = 0.0f;
	stepper->current_accel = 0.0f;
	stepper->jerk = 1.0f;
	stepper->accel = 1.0f;
	stepper->decel = 1.0f;
}

void Stepper_SetMotionProfile(Stepper_t *stepper, Stepper_Profile_t profile)
{
	if ((stepper == 0) || (profile > STEPPER_PROFILE_S_CURVE))
	{
		return;
	}
	stepper->profile = profile;
}

Stepper_Profile_t Stepper_GetMotionProfile(const Stepper_t *stepper)
{
	if (stepper == 0)
	{
		return STEPPER_PROFILE_TRAPEZOID;
	}
	return stepper->profile;
}

void Stepper_EN(GPIO_TypeDef *en_port, uint16_t en_pin,Stepper_ENN FEN){
	if(FEN)
		HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);
}

void Stepper_Set_Speed_Profile(Stepper_t *stepper, float speed, float accel)
{	
	if(accel < 0)return;
	if (stepper->run_mode == MODE_POSITION)
	{
		if (speed < 0 || accel < 0)
			return;
		stepper->accel = accel;
		stepper->decel = accel;
		stepper->max_speed = speed;
		if (speed < 1.0f)
		{
			stepper->min_delay = 10000000; // 10s
		}
		stepper->min_delay = (uint32_t)(stepper->timer_freq / fabsf(speed));
	}
	else if (stepper->run_mode == MODE_SPEED)
	{
		if ((stepper->state != STOP) &&
			(stepper->current_speed * speed < 0.0f))
		{
			/* A direction change is always a clean stop then restart. */
			Stepper_Stop(stepper);
		}
		if (speed > 0.0f)
		{
			stepper->direction = DIR_REVERSE;
		}
		else
		{
			stepper->direction = DIR_FORWARD;
		}
		HAL_GPIO_WritePin(stepper->dir_port, stepper->dir_pin, (GPIO_PinState)stepper->direction);
		stepper->accel = accel;
		stepper->target_speed = speed;
		stepper->jerk = accel / STEPPER_SCURVE_RAMP_TIME_S;
		if (stepper->jerk < 1.0f)
		{
			stepper->jerk = 1.0f;
		}
		if (Stepper_AbsF(speed) >= STEPPER_MIN_SPEED)
		{
			stepper->min_delay = Stepper_SpeedToDelay(stepper, speed);
		}
	}
}

// 位置模式移动
void Stepper_Move(Stepper_t *stepper, int32_t relative_steps)
{
	if (stepper->run_mode == MODE_SPEED)
		return;

	if (stepper->state != STOP || relative_steps == 0)
		return;

	uint32_t requested_steps = labs(relative_steps);

	// 方向
	if (relative_steps > 0)
	{
		stepper->direction = DIR_REVERSE;
	}
	else
	{
		stepper->direction = DIR_FORWARD;
	}
	HAL_GPIO_WritePin(stepper->dir_port, stepper->dir_pin, (GPIO_PinState)stepper->direction);

	stepper->steps_to_move = requested_steps + 1U;
	stepper->target_pos = stepper->current_pos + relative_steps;

	uint32_t decel_steps_needed = (stepper->max_speed * stepper->max_speed) / (2.0f * stepper->decel); // 加速步数及减速步数

	if (stepper->steps_to_move / 2 < decel_steps_needed)
	{ // 无法加速到最高速
		stepper->decel_start_step = stepper->steps_to_move / 2;
	}
	else
	{
		stepper->decel_start_step = stepper->steps_to_move - decel_steps_needed;
	}

	stepper->irq_call_count = 0;
	stepper->accel_count = 0;
	stepper->state = ACCEL;

	uint32_t initial_delay = (uint32_t)(0.676f * sqrtf(2.0f / stepper->accel) * stepper->timer_freq);
	if (initial_delay == 0)
		initial_delay = 1;
	if (initial_delay > 1000000)
		initial_delay = 1000000;

	stepper->step_delay = initial_delay;
	__HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->step_delay);
	__HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, stepper->step_delay / 2);

	// 手动、分步启动
	__HAL_TIM_CLEAR_IT(stepper->htim, TIM_IT_UPDATE);
	HAL_TIM_PWM_Start(stepper->htim, stepper->tim_channel);
	__HAL_TIM_ENABLE_IT(stepper->htim, TIM_IT_UPDATE);
}

void Stepper_Run_Speed(Stepper_t *stepper)
{
	if (stepper->run_mode != MODE_SPEED)
		return;

	if (stepper->state != STOP)
		return;

	stepper->accel_count = 0;
	stepper->state = ACCEL;

	uint32_t initial_delay = (uint32_t)(0.676f * sqrtf(2.0f / stepper->accel) * stepper->timer_freq);
	if (initial_delay > 600000)
		initial_delay = 600000;
	else if (initial_delay < 100)
		initial_delay = 100;

	stepper->step_delay = initial_delay;
	stepper->current_speed = (stepper->target_speed >= 0.0f) ?
		(stepper->timer_freq / (float)stepper->step_delay) :
		-(stepper->timer_freq / (float)stepper->step_delay);
	stepper->current_accel = 0.0f;

	__HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->step_delay);
	__HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, stepper->step_delay / 2);

	// 手动、分步启动
	__HAL_TIM_CLEAR_IT(stepper->htim, TIM_IT_UPDATE);
	HAL_TIM_PWM_Start(stepper->htim, stepper->tim_channel);
	__HAL_TIM_ENABLE_IT(stepper->htim, TIM_IT_UPDATE);
}

// 5. 强制停止
void Stepper_Stop(Stepper_t *stepper)
{	
	if (stepper->state == STOP || stepper->run_mode != MODE_SPEED)
		return;
	stepper->state = STOP;
	stepper->step_delay = 600000;
	stepper->current_speed = 0;
	__HAL_TIM_DISABLE_IT(stepper->htim, TIM_IT_UPDATE);
	HAL_TIM_PWM_Stop(stepper->htim, stepper->tim_channel);
	__HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, 0);
}

// 6. 中断处理函数 (算法核心)
// 6. 中断处理函数 (算法核心)
void Stepper_TIM_IRQ_Handler(Stepper_t *stepper)
{       
    if (stepper->direction == DIR_FORWARD) {
        stepper->current_pos++;
    } else {
        stepper->current_pos--;
    }

    uint32_t new_delay = stepper->step_delay;

		if (stepper->run_mode == MODE_SPEED) {
			if (stepper->profile == STEPPER_PROFILE_S_CURVE)
			{
				if (Stepper_UpdateSCurveAtPulse(stepper, &new_delay) == 0U)
				{
					return;
				}
			}
			else {
				float target_speed_abs = fabsf(stepper->target_speed);
				float current_speed_abs = fabsf(stepper->current_speed);
				float dt = (float)stepper->step_delay / stepper->timer_freq;
				float speed_change = stepper->accel * dt;

				if (target_speed_abs < 1.0f) {
						Stepper_Stop(stepper);
						return;
				}

				if (current_speed_abs < target_speed_abs) {
						current_speed_abs += speed_change;
						if (current_speed_abs >= target_speed_abs) {
								current_speed_abs = target_speed_abs;
								stepper->state = RUN;
						} else {
								stepper->state = ACCEL;
						}
				} else if (current_speed_abs > target_speed_abs) {
						current_speed_abs -= speed_change;
						if (current_speed_abs <= target_speed_abs) {
								current_speed_abs = target_speed_abs;
								stepper->state = RUN;
						} else {
								stepper->state = DECEL;
						}
				}

				if (current_speed_abs < 1.0f) {
						current_speed_abs = 1.0f;
				}

				stepper->current_speed = (stepper->target_speed >= 0.0f) ?
						current_speed_abs : -current_speed_abs;
				new_delay = (uint32_t)(stepper->timer_freq / current_speed_abs);
			}
		}
    else if (stepper->run_mode == MODE_POSITION)
    {
        stepper->irq_call_count++;
        if (stepper->irq_call_count >= stepper->steps_to_move) {
            stepper->state = STOP;
            __HAL_TIM_DISABLE_IT(stepper->htim, TIM_IT_UPDATE);
            HAL_TIM_PWM_Stop(stepper->htim, stepper->tim_channel);
            return;
        }
        uint32_t steps_remaining = stepper->steps_to_move - stepper->irq_call_count; 

        switch (stepper->state) {
            case ACCEL:
                stepper->accel_count++;
                new_delay = stepper->step_delay - (2 * stepper->step_delay) / (4 * stepper->accel_count + 1);
                if (new_delay < stepper->min_delay) {
                    new_delay = stepper->min_delay;
                    stepper->state = RUN;
                }
                if (stepper->irq_call_count >= stepper->decel_start_step) {
                    stepper->state = DECEL;
                }
                break;
            case RUN:
                new_delay = stepper->min_delay;
                if (stepper->irq_call_count >= stepper->decel_start_step) {
                    stepper->state = DECEL;
                }
                break;
            case DECEL:
                new_delay = stepper->step_delay + (2 * stepper->step_delay) / (4 * steps_remaining + 1);
                break;
            default:
                new_delay = stepper->step_delay;
                break;
        }
    }

    stepper->step_delay = new_delay;
    __HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->step_delay);
    __HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, stepper->step_delay / 2);
}


