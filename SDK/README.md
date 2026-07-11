# Step42 Low-Level UART SDK

This SDK only builds and parses binary protocol frames. It does not call any
platform UART API. You build a frame, then send it with your own UART function.

## Files

```text
step42_protocol.h
step42_protocol.c
example_minimal.c
step42_stm32_hal.h
step42_stm32_hal.c
example_stm32_hal.c
```

Add `step42_protocol.c/.h` to every project. For STM32 HAL projects, also add
`step42_stm32_hal.c/.h`.

## Minimal Usage

```c
uint8_t tx[STEP42_MAX_FRAME_SIZE];
uint8_t seq = 0;
uint16_t len;

len = Step42_BuildEnable(tx, sizeof(tx), ++seq, 1);
uart_send(tx, len);

len = Step42_BuildSetMode(tx, sizeof(tx), ++seq, STEP42_MODE_CLOSED_POSITION);
uart_send(tx, len);

len = Step42_BuildSetAngle(tx, sizeof(tx), ++seq, 90.0f);
uart_send(tx, len);
```

`seq` can be incremented for every command. The response carries the same `seq`.
For very simple one-way control, it can also be fixed at `0`.

## STM32 HAL Example

Simple wrapper usage:

```c
#include "step42_stm32_hal.h"

extern UART_HandleTypeDef huart2;

Step42_STM32HAL_Handle motor;

void app_init(void)
{
    Step42_STM32HAL_Init(&motor, &huart2, 100);
}

void app_run(void)
{
    Step42_STM32HAL_Enable(&motor, 1);
    Step42_STM32HAL_SetMode(&motor, STEP42_MODE_CLOSED_POSITION);
    Step42_STM32HAL_SetAngle(&motor, 90.0f);
}
```

Low-level usage:

```c
uint8_t tx[STEP42_MAX_FRAME_SIZE];
uint16_t len;

len = Step42_BuildSetAngle(tx, sizeof(tx), 1, 90.0f);
HAL_UART_Transmit(&huart1, tx, len, 100);
```

## TI DL Example

```c
uint8_t tx[STEP42_MAX_FRAME_SIZE];
uint16_t len;

len = Step42_BuildSetAngle(tx, sizeof(tx), 1, 90.0f);
for (uint16_t i = 0; i < len; i++)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, tx[i]);
}
```

## Linux C Example

```c
uint8_t tx[STEP42_MAX_FRAME_SIZE];
uint16_t len;

len = Step42_BuildSetAngle(tx, sizeof(tx), 1, 90.0f);
write(fd, tx, len);
```

## Receive Parser

Feed received bytes into `Step42_ParseByte`.

```c
Step42_Parser parser;
Step42_Response resp;

Step42_ParserInit(&parser);

if (Step42_ParseByte(&parser, rx_byte, &resp) == STEP42_PARSE_FRAME)
{
    uint8_t status;
    if (Step42_ResponseStatus(&resp, &status) == 0 && status == STEP42_STATUS_OK)
    {
        /* command accepted */
    }
}
```

## Control Modes

```text
STEP42_MODE_OPEN_POSITION    fixed open-loop step move to angle
STEP42_MODE_CLOSED_POSITION  encoder PID position control
STEP42_MODE_OPEN_SPEED       open-loop continuous speed
STEP42_MODE_TRACK_PID        external-error tracking PID, fixed 32 microsteps
```

Set the mode first, then send angle or speed:

```c
len = Step42_BuildSetMode(tx, sizeof(tx), ++seq, STEP42_MODE_OPEN_SPEED);
uart_send(tx, len);

len = Step42_BuildSetSpeed(tx, sizeof(tx), ++seq, 1000, 3000);
uart_send(tx, len);
```

## Tracking PID

The tracking controller accepts either one error loop or a cascade controller.
For cascade control, `primary_error` is normally the slower vision error and
`secondary_feedback` is the faster gyro/rate feedback.

```c
len = Step42_BuildSetMode(tx, sizeof(tx), ++seq, STEP42_MODE_TRACK_PID);
uart_send(tx, len);

/* mode=1 single loop, speed=100%, accel=100%, loop=1 kHz */
len = Step42_BuildTrackConfig(tx, sizeof(tx), ++seq, 1, 100, 100, 1);
uart_send(tx, len);

len = Step42_BuildTrackPid(tx, sizeof(tx), ++seq, 0, 180.0f, 0.2f, 0.0f);
uart_send(tx, len);

len = Step42_BuildTrackInput(tx, sizeof(tx), ++seq, vision_error, gyro_feedback);
uart_send(tx, len);
```

The UART data stream is disabled after reset. Send `Step42_BuildSetStream()`
only when continuous status telemetry is needed.

Chopper mode can be selected without changing the current-reference source:

```c
/* 0: quiet StealthChop, 1: higher-torque SpreadCycle */
len = Step42_BuildSetChopperMode(tx, sizeof(tx), ++seq, 1);
uart_send(tx, len);
```

## DIP Transport Modes

The two switch inputs are sampled once during startup. With `MS1` as the high
bit and `MS2` as the low bit:

```text
MS1 MS2 = 1 1  ERROR UART
MS1 MS2 = 0 1  PWM input
MS1 MS2 = 1 0  CAN
MS1 MS2 = 0 0  Factory Debug UART
```
