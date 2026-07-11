#ifndef STEP42_PWM_INPUT_H
#define STEP42_PWM_INPUT_H

#include "main.h"

void Step42PwmInput_Init(void);
void Step42PwmInput_Process(void);
void Step42PwmInput_CaptureCallback(TIM_HandleTypeDef *htim);

#endif
