/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_4
#define LED_RED_GPIO_Port GPIOA
#define MT6701_SCK_Pin GPIO_PIN_5
#define MT6701_SCK_GPIO_Port GPIOA
#define MT6701_MISO_Pin GPIO_PIN_6
#define MT6701_MISO_GPIO_Port GPIOA
#define MT6701_CSN_Pin GPIO_PIN_7
#define MT6701_CSN_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_0
#define LED_BLUE_GPIO_Port GPIOB
#define LED_WHITE_Pin GPIO_PIN_1
#define LED_WHITE_GPIO_Port GPIOB
#define tmc_EN_Pin GPIO_PIN_14
#define tmc_EN_GPIO_Port GPIOB
#define tmc_DIR_Pin GPIO_PIN_15
#define tmc_DIR_GPIO_Port GPIOB
#define tmc_STEP_Pin GPIO_PIN_8
#define tmc_STEP_GPIO_Port GPIOA
#define TMC2209_TX_Pin GPIO_PIN_9
#define TMC2209_TX_GPIO_Port GPIOA
#define TMC2209_RX_Pin GPIO_PIN_10
#define TMC2209_RX_GPIO_Port GPIOA
#define PWM_IN_Pin GPIO_PIN_15
#define PWM_IN_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOB
#define MS1_Pin GPIO_PIN_6
#define MS1_GPIO_Port GPIOB
#define MS2_Pin GPIO_PIN_7
#define MS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
