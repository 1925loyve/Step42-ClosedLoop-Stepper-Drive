
#ifndef LED_H
#define LED_H

#include "main.h"

#define led_red_on HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)
#define led_red_off HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
#define led_red_tog HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
#define led_blue_on HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET)
#define led_blue_off HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET)
#define led_blue_tog HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
#define led_white_on HAL_GPIO_WritePin(LED_WHITE_GPIO_Port,LED_WHITE_Pin,GPIO_PIN_SET)
#define led_white_off HAL_GPIO_WritePin(LED_WHITE_GPIO_Port,LED_WHITE_Pin,GPIO_PIN_RESET)
#define led_white_tog HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);

void led_Init(void);

#endif
