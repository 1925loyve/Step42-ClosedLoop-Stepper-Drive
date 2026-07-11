#include "Step42App.h"
#include "Step42Comms.h"
#include "Step42Control.h"
#include "Key.h"
#include "MT6701.h"
#include "Tmc2209.h"
#include "delay.h"
#include "led.h"
#include "Step42BoardMode.h"
#include "Step42Sensors.h"
#include "Step42PwmInput.h"

extern TIM_HandleTypeDef htim6;

void Step42App_Init(void)
{
  uint8_t tmc_addr = 0xFFU;

  DWT_Init();
  MT6701_Init();
  Step42BoardMode_Init();
  Step42Sensors_Init();
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

  Step42Control_Init();
  Step42PwmInput_Init();
  HAL_TIM_Base_Start_IT(&htim6);
}

void Step42App_Loop(void)
{
  Step42Sensors_Process();
  Step42Control_SetThermalLockout(
      (Step42Sensors_GetFlags() & STEP42_SENSOR_TEMP_SHUTDOWN) != 0U);
  Step42Comms_ProcessRx();
  Step42PwmInput_Process();
  Step42Comms_SendStreamQueryIfDue();
  Step42Control_HandleKey(Key_Scan());
}

void Step42App_TIMPeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    if (Step42Control_TimerTick1kHz() != 0U)
    {
      /* Telemetry is sent from the main loop to keep this ISR deterministic. */
    }
  }
}

void Step42App_UARTErrorCallback(UART_HandleTypeDef *huart)
{
  Step42Comms_OnUartError(huart);
}

void Step42App_TIMCaptureCallback(TIM_HandleTypeDef *htim)
{
  Step42PwmInput_CaptureCallback(htim);
}
