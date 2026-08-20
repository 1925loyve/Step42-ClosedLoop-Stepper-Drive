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

static uint8_t tmc2209_ready = 0U;

static void Step42App_UpdateIndicators(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t temp_flags = Step42Sensors_GetFlags();
  uint8_t fault = (uint8_t)((tmc2209_ready == 0U) ||
                            (Step42Control_GetEncoderStatus() != MT6701_OK));

  /* HAL_GetTick is SysTick based; the indicator timing is fully nonblocking. */
  (void)HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin,
                          ((now % 1000U) < 500U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if ((temp_flags & STEP42_SENSOR_TEMP_SHUTDOWN) != 0U)
  {
    led_red_on;
  }
  else if ((Step42Control_IsStallFault() != 0U) || (fault != 0U))
  {
    (void)HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                            ((now % 200U) < 100U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if ((temp_flags & STEP42_SENSOR_TEMP_WARNING) != 0U)
  {
    (void)HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                            ((now % 500U) < 250U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    led_red_off;
  }

  if (Step42Control_IsProgramRunning() != 0U)
  {
    led_blue_on;
  }
  else if (Step42Control_IsExternalTaskActive() != 0U)
  {
    (void)HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                            ((now % 1000U) < 500U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    led_blue_off;
  }
}

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
    tmc2209_ready = 1U;
  }

  Step42Control_Init();
  Step42PwmInput_Init();
  Step42App_UpdateIndicators();
  HAL_TIM_Base_Start_IT(&htim6);
}

void Step42App_Loop(void)
{
  if (Step42BoardMode_Process() != 0U)
  {
    Step42Control_SetUserEnable(0U);
    Step42Comms_OnBoardModeChanged();
  }
  Step42Sensors_Process();
  Step42Control_SetThermalLockout(
      (Step42Sensors_GetFlags() & STEP42_SENSOR_TEMP_SHUTDOWN) != 0U);
  Step42Comms_ProcessRx();
  Step42PwmInput_Process();
  Step42Control_ProcessProgram();
  Step42Comms_SendStreamQueryIfDue();
  Step42Control_HandleKey(Key_Scan());
  Step42App_UpdateIndicators();
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

void Step42App_UARTRxCompleteCallback(UART_HandleTypeDef *huart)
{
  Step42Comms_OnUartRxComplete(huart);
}

void Step42App_FDCANRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
  if ((hfdcan->Instance == FDCAN1) &&
      ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U))
  {
    Step42Comms_OnCanRxFifo0();
  }
}

void Step42App_TIMCaptureCallback(TIM_HandleTypeDef *htim)
{
  Step42PwmInput_CaptureCallback(htim);
}
