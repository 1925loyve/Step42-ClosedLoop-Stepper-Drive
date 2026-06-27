#include "Step42App.h"
#include "Step42Comms.h"
#include "Step42Control.h"
#include "Key.h"
#include "MT6701.h"
#include "Tmc2209.h"
#include "delay.h"
#include "led.h"

extern TIM_HandleTypeDef htim6;

void Step42App_Init(void)
{
  uint8_t tmc_addr = 0xFFU;

  DWT_Init();
  MT6701_Init();
  Step42Comms_Init();

  led_red_off;
  led_blue_off;
  led_white_off;

  if (Tmc2209_InitAuto(&tmc_addr) != 0U)
  {
    led_blue_on;
  }
  else
  {
    led_red_on;
  }

  HAL_TIM_Base_Start_IT(&htim6);
  Step42Control_Init();
}

void Step42App_Loop(void)
{
  Step42Comms_ProcessRx();
  Step42Control_HandleKey(Key_Scan());
}

void Step42App_TIMPeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    if (Step42Control_TimerTick1kHz() != 0U)
    {
      Step42Comms_SendStreamQueryIfDue();
    }
  }
}

void Step42App_UARTErrorCallback(UART_HandleTypeDef *huart)
{
  Step42Comms_OnUartError(huart);
}
