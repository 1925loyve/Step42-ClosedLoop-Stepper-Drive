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
static uint32_t mode_announce_start_ms = 0U;
static uint32_t key_feedback_start_ms = 0U;
static uint8_t key_feedback_error = 0U;
static uint8_t key_feedback_pending = 0U;

#define KEY_FEEDBACK_DURATION_MS (850U)

static uint8_t Step42App_IsPulseOn(uint32_t elapsed, uint8_t count)
{
  uint8_t index;

  for (index = 0U; index < count; index++)
  {
    uint32_t start = (uint32_t)index * 220U;

    if ((elapsed >= start) && (elapsed < (start + 110U)))
    {
      return 1U;
    }
  }
  return 0U;
}

static void Step42App_NotifyKey(int8_t key_event)
{
  key_feedback_start_ms = HAL_GetTick();
  key_feedback_error = 0U;
  key_feedback_pending = 1U;

  if ((key_event == KEY_EVENT_HOME_SHORT) &&
      (Step42Control_IsZeroValid() == 0U))
  {
    key_feedback_error = 1U;
  }
}

static void Step42App_UpdateIndicators(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t sensor_flags = Step42Sensors_GetFlags();
  uint8_t fatal_fault = (uint8_t)((tmc2209_ready == 0U) ||
                                  (Step42Control_GetEncoderStatus() != MT6701_OK));
  uint8_t mode_announcing =
      ((uint32_t)(now - mode_announce_start_ms) < 3200U) ? 1U : 0U;
  uint8_t key_feedback_active;
  uint8_t key_feedback_on =
      Step42App_IsPulseOn((uint32_t)(now - key_feedback_start_ms), 3U);

  if ((key_feedback_pending != 0U) &&
      ((uint32_t)(now - key_feedback_start_ms) >= KEY_FEEDBACK_DURATION_MS))
  {
    key_feedback_pending = 0U;
  }
  key_feedback_active = key_feedback_pending;

  /* Blue: boot/mode acknowledgement first, then a low-duty health heartbeat. */
  if (fatal_fault != 0U)
  {
    (void)HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                            ((now % 250U) < 125U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if (mode_announcing != 0U)
  {
    uint8_t pulses = (uint8_t)Step42BoardMode_Get() + 1U;
    uint32_t cycle = (uint32_t)(now - mode_announce_start_ms) % 1000U;
    (void)HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                            Step42App_IsPulseOn(cycle, pulses) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    (void)HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                            ((now % 1000U) < 80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  /* Red is reserved for a condition that needs the user's attention. */
  if ((sensor_flags & STEP42_SENSOR_TEMP_SHUTDOWN) != 0U)
  {
    led_red_on;
  }
  else if ((sensor_flags & STEP42_SENSOR_TEMP_WARNING) != 0U)
  {
    (void)HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                            ((now % 500U) < 250U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if (fatal_fault != 0U)
  {
    (void)HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                            ((now % 250U) < 125U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if ((key_feedback_active != 0U) && (key_feedback_error != 0U))
  {
    (void)HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                            (key_feedback_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    led_red_off;
  }

  /* White distinguishes disabled, holding torque, and active motion. */
  if ((sensor_flags & STEP42_SENSOR_TEMP_SHUTDOWN) != 0U)
  {
    led_white_off;
  }
  else if ((key_feedback_active != 0U) && (key_feedback_error == 0U))
  {
    (void)HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin,
                            (key_feedback_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if (Step42Control_IsMotionActive() != 0U)
  {
    led_white_on;
  }
  else if (Step42Control_IsUserEnabled() != 0U)
  {
    (void)HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin,
                            ((now % 1200U) < 130U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    led_white_off;
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
  mode_announce_start_ms = HAL_GetTick();
  Step42App_UpdateIndicators();
  HAL_TIM_Base_Start_IT(&htim6);
}

void Step42App_Loop(void)
{
  if (Step42BoardMode_Process() != 0U)
  {
    Step42Control_SetUserEnable(0U);
    Step42Comms_OnBoardModeChanged();
    mode_announce_start_ms = HAL_GetTick();
  }
  Step42Sensors_Process();
  Step42Control_SetThermalLockout(
      (Step42Sensors_GetFlags() & STEP42_SENSOR_TEMP_SHUTDOWN) != 0U);
  Step42Comms_ProcessRx();
  Step42PwmInput_Process();
  Step42Comms_SendStreamQueryIfDue();
  {
    int8_t key_event = Key_Scan();

    if (key_event != 0)
    {
      Step42Control_HandleKey(key_event);
      Step42App_NotifyKey(key_event);
    }
  }
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

void Step42App_TIMCaptureCallback(TIM_HandleTypeDef *htim)
{
  Step42PwmInput_CaptureCallback(htim);
}
