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
step42_can_transport.h
step42_can_transport.c
example_stm32_fdcan_hal.c
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
STEP42_MODE_EXTERNAL_POSITION external continuous-position target with independent tuning
```

Set the mode first, then send angle or speed:

```c
len = Step42_BuildSetMode(tx, sizeof(tx), ++seq, STEP42_MODE_OPEN_SPEED);
uart_send(tx, len);

len = Step42_BuildSetSpeed(tx, sizeof(tx), ++seq, 1000, 3000);
uart_send(tx, len);
```

The pulse-rate ceiling follows the selected microstep value:

```text
limit = min(5000 * microsteps, 30000) pulse/s
1 / 2 / 4 / 8+ microsteps -> 5000 / 10000 / 20000 / 30000 pulse/s
```

Call `Step42_GetPulseSpeedLimit(microsteps)` before building a speed or step
command when the application accepts user-entered values. Frame builders only
enforce the 30000 pulse/s hard ceiling; the drive enforces the current
microstep-specific limit and returns a range error when it is exceeded.

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

/* Advanced algorithms are explicitly opt-in. This example enables filtering only. */
len = Step42_BuildTrackAdvanced(
    tx, sizeof(tx), ++seq, 0.25f, 1.0f, 1.0f, 10.0f,
    0, 250, 0.08f, 0.08f, 0.08f, STEP42_TRACK_ADV_FILTER);
uart_send(tx, len);

/* Send this once per sensor/camera sample. Values are raw signed integers. */
len = Step42_BuildSignalInput(tx, sizeof(tx), ++seq,
                              target_detected, custom_signal_1,
                              custom_signal_2, vision_error, gyro_feedback);
uart_send(tx, len);

/* STM32 HAL wrapper */
Step42_STM32HAL_SetSignals(&motor, target_detected, custom_signal_1,
                           custom_signal_2, vision_error, gyro_feedback);
```

The fixed request is 11 bytes: `A5 5A 05 27 SEQ FLAGS ERROR1_L ERROR1_H
ERROR2_L ERROR2_H CRC8`. The errors are little-endian signed 16-bit integers;
they are not floating-point values and are not multiplied by 100. `FLAGS`
bits 0, 1 and 2 are `target_detected`, `custom1` and `custom2`. Send a frame
for every sensor/camera sample, including `target_detected=0` with both errors
set to zero when the target is absent. The flags can drive offline-program
branches. In tracking PID mode the numeric fields also update the PID input.
UART and CAN carry the same binary command payload.

The UART data stream is disabled after reset. Send `Step42_BuildSetStream()`
only when continuous status telemetry is needed.

## External Continuous Position

This mode is intended for a camera or another MCU that calculates a continuous
position target every frame. It shares the board's single global zero, but has
independent microsteps, PID, speed limit, soft range, timeout and target-loss
behavior. Set the global zero once before entering this mode.

```c
Step42_STM32HAL_SetMode(&motor, STEP42_MODE_EXTERNAL_POSITION);

Step42_STM32HAL_SetExternalPositionConfig(
    &motor, 32, 3000, 180.0f, 0.2f, 0.0f, 5000, 0.08f,
    -80.0f, 80.0f, 200, STEP42_EXTERNAL_LOSS_HOLD);

/* Call once per external-controller sample. */
Step42_STM32HAL_SetExternalPositionTarget(&motor, target_deg, target_valid);
```

If `target_valid` becomes zero or frames stop for `timeout_ms`, the selected
loss action holds the current position, returns to global zero, or disables the
motor. Runtime tuning changes RAM. Read either copy with
`Step42_STM32HAL_QueryExternalPosition(..., STEP42_CONFIG_SOURCE_RAM/FLASH)`;
call `Step42_STM32HAL_SaveRuntimeConfigScope(&motor, STEP42_CONFIG_SAVE_EXTERNAL)`
to persist only this page. Use `STEP42_CONFIG_SAVE_TRACKING` for the internal
tracking page, or the legacy `Step42_STM32HAL_SaveRuntimeConfig()` to persist
all persistent settings at once. Every Flash save is explicitly rate-limited by
the firmware.

Chopper mode can be selected without changing the current-reference source:

```c
/* 0: quiet StealthChop, 1: higher-torque SpreadCycle */
len = Step42_BuildSetChopperMode(tx, sizeof(tx), ++seq, 1);
uart_send(tx, len);
```

Hold and run current use the TMC2209 `IHOLD`/`IRUN` current-scale values, not
ampere units. `hold_current` is `0..31`, `run_current` is `1..31`, and hold
must not exceed run. The actual coil current still depends on `VREF`, the
sense resistors, and the TMC2209 current formula.

```c
/* IHOLD=4, IRUN=12 */
len = Step42_BuildSetCurrent(tx, sizeof(tx), ++seq, 4, 12);
uart_send(tx, len);

/* STM32 HAL wrapper */
Step42_STM32HAL_SetCurrent(&motor, 4, 12);
```

Protocol version 20 adds explicit advanced-feature enable bits, separate RAM
and Flash queries for tracking/external-position configuration, and the shared
global-zero external continuous-position mode. Protocol version 19 makes
signal input `0x27` the only PID/vision input and
uses two raw signed 16-bit errors. Legacy command `0x13` and newline-delimited
ASCII input are removed. Protocol version 18 appended the original signal
input and expanded offline programs to 32 steps with drive-state,
conditional-branch, and goto actions.
Protocol version 17 appended `hold_current` and `run_current` to Query response
payload bytes 69 and 70. Older Query replies remain decodable and use the
default values `4 / 12` when those bytes are absent.

## DIP Transport Modes

The two switch inputs are sampled once during startup. With `MS1` as the high
bit and `MS2` as the low bit:

```text
MS1 MS2 = 1 1  ERROR UART
MS1 MS2 = 0 1  PWM input
MS1 MS2 = 1 0  CAN
MS1 MS2 = 0 0  Factory Debug UART
```

## PWM Position Input

With the DIP switches at `0 1`, the board accepts a servo-style PWM position
input on `PA15 / TIM2_CH1`. Use a 50 Hz signal with a 1000 to 2000 us high
pulse width: 1000 us maps to the stored software zero and 2000 us maps to the
equivalent 360-degree endpoint. Intermediate widths map linearly across one
mechanical turn. PWM uses a cyclic single-turn target, so crossing 0/360 never
causes an unintended full-turn correction. At power-on the board starts with
32 microsteps and default closed-position PID values. During one powered
session, PWM uses any position PID/acceleration changes made through UART.

## Classic CAN

CAN uses the same Step42 binary frames as UART. Add `step42_can_transport.c/.h`
to the controller project, build a normal command with `Step42_Build...`, then
split it with `Step42_CAN_Encode()` and transmit the resulting packets using
standard CAN ID `STEP42_CAN_COMMAND_ID(node_id)`. The drive replies on
`STEP42_CAN_RESPONSE_ID(node_id)`.

```c
uint8_t raw[STEP42_MAX_FRAME_SIZE];
Step42_CAN_Packet packets[STEP42_CAN_MAX_SEGMENTS];
uint16_t raw_len;
uint8_t packet_count;

raw_len = Step42_BuildSetAngle(raw, sizeof(raw), 1, 90.0f);
packet_count = Step42_CAN_Encode(raw, raw_len, packets, STEP42_CAN_MAX_SEGMENTS);
/* Send packets[0..packet_count-1] with STEP42_CAN_COMMAND_ID(1). */
```

The drive runs Classic CAN at 500 kbit/s. Node IDs are `1..31`, default `1`;
the default command/response IDs are therefore `0x301` and `0x341`.
Use `Step42_BuildSetCanNodeId()` once to write a new ID to Flash. When changing
an ID through CAN, listen for the acknowledgement at the new response ID.
`example_stm32_fdcan_hal.c` contains a complete transmit helper. To decode replies, feed each received `STEP42_CAN_RESPONSE_ID(node_id)`
packet into `Step42_CAN_ReassemblerPush()`, then pass the completed raw frame
byte-by-byte to the regular `Step42_ParseByte()` parser.

In CAN mode, the drive's Debug UART remains available to the PC tuning tool
for Query, telemetry streaming, and tracking PID configuration. It is
deliberately restricted from issuing position, speed, step, or home motion
commands; the CAN controller remains the only full motion-control source.
