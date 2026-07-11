#ifndef STEP42_BOARD_MODE_H
#define STEP42_BOARD_MODE_H

#include <stdint.h>

typedef enum
{
    STEP42_BOARD_FACTORY_DEBUG = 0,
    STEP42_BOARD_PWM = 1,
    STEP42_BOARD_CAN = 2,
    STEP42_BOARD_UART = 3
} Step42BoardMode_t;

void Step42BoardMode_Init(void);
Step42BoardMode_t Step42BoardMode_Get(void);

#endif
