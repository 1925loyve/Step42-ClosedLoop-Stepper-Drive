# Step42 Low-Level UART SDK

This SDK only builds and parses binary protocol frames. It does not call any
platform UART API. You build a frame, then send it with your own UART function.

## Files

```text
step42_protocol.h
step42_protocol.c
example_minimal.c
```

Add `step42_protocol.c/.h` to the user's project.

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
STEP42_MODE_CLOSED_SPEED     encoder feedback weak speed control
```

Set the mode first, then send angle or speed:

```c
len = Step42_BuildSetMode(tx, sizeof(tx), ++seq, STEP42_MODE_CLOSED_SPEED);
uart_send(tx, len);

len = Step42_BuildSetSpeed(tx, sizeof(tx), ++seq, 1000, 3000);
uart_send(tx, len);
```
