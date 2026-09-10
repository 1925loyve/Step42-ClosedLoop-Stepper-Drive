# Step42 Control Protocol

Error UART (USART2) and Debug UART (USART3): 115200, 8 data bits, no parity,
1 stop bit. The upper computer identifies which one it is connected to from
the first Query response.

## Wiring

- UART uses 3.3 V TTL levels: host `TX -> drive RX`, host `RX -> drive TX`,
  and host `GND -> drive GND`. Do not connect RS-232 levels or 5 V logic.
- DIP inputs are pulled high; a switch connected to ground reads as `0`.
  `MS1 MS2`: `00=Factory Debug`, `01=PWM`, `10=CAN`, `11=UART`.
- The bipolar motor connector is `A+ / A- / B+ / B-`. With power removed,
  use resistance measurement to identify the two coil pairs. Never hot-plug
  the motor connector.
- The MT6701 requires a radially magnetized magnet centered over the sensor.
- Connect motor power with the polarity and voltage range marked on the board.

The firmware command interface uses this binary protocol only. The PC tuning
tool and MCU/Linux SDK use the same frame format.

## Unified Signal And Error Input

The old newline-delimited text input is not supported. Vision modules, MCUs
and Linux hosts must send command `0x27` as a fixed 11-byte binary frame:

```text
A5 5A 05 27 SEQ FLAGS ERROR1_L ERROR1_H ERROR2_L ERROR2_H CRC8
```

- `SEQ`: increment for every frame and wrap from 255 to 0.
- `FLAGS bit0`: `target_detected`.
- `FLAGS bit1`: user-defined signal `custom1`.
- `FLAGS bit2`: user-defined signal `custom2`.
- `FLAGS bit3..7`: reserved and must be zero.
- `ERROR1`, `ERROR2`: raw little-endian signed `int16_t`; no decimal scaling.
- Single-loop PID uses `ERROR1` and sets `ERROR2=0`.
- Cascade PID normally uses `ERROR1` as the outer vision/position error and
  `ERROR2` as the inner speed or angular-rate feedback.
- Send one frame for every camera/sensor sample. If no target is detected,
  send `target_detected=0` and normally set both errors to zero.
- The same command remains accepted while an internal tracking instruction is
  running, so program branches can react to `target/custom1/custom2` while the
  numeric fields refresh the PID input.

Example for `SEQ=1`, `target=1`, `custom1=0`, `custom2=1`, `ERROR1=120`,
`ERROR2=-35`:

```text
A5 5A 05 27 01 05 78 00 DD FF 0B
```

Successful UART response:

```text
5A A5 01 A7 01 00 5D
```

`0x1F Tracking direction` uses request `u8 reverse` (`0=normal`, `1=reverse`).
The upper computer normally sends it through Debug UART.

## Frame

Request:

```text
A5 5A LEN CMD SEQ PAYLOAD CRC8
```

Response:

```text
5A A5 LEN CMD|80 SEQ PAYLOAD CRC8
```

- `LEN`: payload byte count, maximum 72.
- `SEQ`: host sequence number. Increment it for every command and match it in
  the response.
- Multi-byte integers are little-endian.
- `CRC8`: CRC-8/poly `0x07`, init `0x00`, over `LEN CMD SEQ PAYLOAD`.

Status byte:

```text
0x00 OK
0x01 bad length
0x02 value out of range
0x03 unknown command
```

## Commands

| CMD | Name | Request payload | Response payload |
| --- | --- | --- | --- |
| `0x01` | Ping | none | `status, proto_version` |
| `0x02` | Enable | `u8 enable` | `status` |
| `0x03` | Multi-turn absolute angle | `i32 angle_cdeg`, range `-720000~720000` | `status` |
| `0x04` | Open-loop speed | `i16 speed, u16 accel` | `status` |
| `0x05` | PID | `i16 kp_x100, i16 ki_x100, i16 kd_x100, u16 limit, u16 dead_x1000` | `status` |
| `0x06` | Stream | `u8 enable` | `status`; when enabled, `SEQ=0` telemetry is sent periodically |
| `0x07` | Query | none | complete current configuration and status, including response interface |
| `0x08` | Mode | `u8 mode` | `status` |
| `0x09` | Stop | none | `status` |
| `0x27` | Signals and PID input | `u8 flags, i16 error1, i16 error2` | `status` |
| `0x17` | Relative closed-loop angle | `i32 delta_cdeg`, range `-720000~720000`; resulting target must remain within `-7200°~7200°` | `status` |
| `0x18` | Program mode | `u8 enable` | `status`; accepted by Error and Debug UART |
| `0x19` | Program step | `u8 index, u8 op, u8 flags, u16 speed, u16 accel, u16 duration_ms, i32 value` | `status`; accepted by Error and Debug UART |
| `0x1A` | Save program | `u8 count`, maximum 8 | `status`; commits staged steps to Flash through either UART |
| `0x1B` | Run program | none | `status`; requires program mode and motor enabled |
| `0x1C` | Program query | none | `status, program_mode, program_running, program_count` |
| `0x1D` | Stage program in RAM | `u8 count`, maximum 8 | `status`; makes uploaded steps runnable without a Flash erase |
| `0x20` | Save persistent configuration | none | `status`; saves zero point, home mode, and all tracking PID settings to Flash. Closed-position PID/acceleration stay session-only. |
| `0x21` | Set CAN Node ID | `u8 node_id`, range `1~31` | `status`; immediately saves the Node ID to Flash |

Control modes:

```text
0x00 open-loop position
0x01 closed-loop position
0x02 open-loop speed
0x03 built-in error PID tracking
```

Protocol v6 appends `response_interface` to the Query response: `0=Error/USART2`,
`1=Debug/USART3`, `2=CAN`. The upper computer uses it to choose its waveform
meaning automatically; the host does not need to select an interface type manually.

Protocol v7 further appends `program_mode`, `program_running`, and
`program_count` to Query. While program mode is enabled, external motion
commands (`angle`, `speed`, `relative angle`, `move steps`, `home`, mode change,
parameter writes and drive settings return `DENIED`. Tracking error input is
the sole exception: it is treated as live sensor data and is accepted only
while the current internal instruction has entered tracking PID mode.
Only telemetry/query, enable, stop and program-management commands remain
available. Signal/error input `0x27` is the sensor-data exception and remains
accepted while a program-owned tracking step is active. The program executor
itself may run position, open-step,
speed, delay, home, tracking PID, microstep setup, chopper setup and stop steps
without blocking the control loop. Program step `op` values are: `1` absolute
position, `2` relative position, `3` open-loop steps, `4` speed, `5` delay,
`6` home, `7` tracking PID, `8` stop, `9` microsteps (`value=1/2/.../256`),
and `10` chopper (`flags=0` StealthChop, `1` SpreadCycle). For speed step `4`,
set `flags bit0=1` to interpret `value` as RPM at execution time; the firmware
converts it using the microstep value currently active in that program flow.

Protocol v8 adds `0x1D Program stage`. Upload all `0x19 Program step` frames,
then stage their count with `0x1D` to run the RAM copy repeatedly. `0x1A Save
program` is accepted only after a successful stage and performs the one Flash
erase/write operation. This keeps normal test runs away from Flash endurance.
After power-up, with the motor enabled, a KEY1 long press starts the valid
stored program without requiring program mode. While that offline program is
running, external motion commands are still denied until it completes or stops.

Protocol v13 adds `0x20 Save persistent configuration`. It is an explicit Flash
operation: normal PID writes update RAM only. The command persists the zero
point, home mode, tracking mode/PID/advanced settings, and error direction.
Closed-position PID and acceleration remain session-only. Do not issue it
while the motor is moving.

## CAN Transport

With DIP `10`, the board enables Classic CAN at `500 kbit/s`. It accepts the
same unchanged binary request frame described above, but carries it in 8-byte
CAN packets because a Classic CAN frame cannot carry the complete UART frame.

```text
Host -> drive: standard ID `0x300 + Node ID`
Drive -> host: standard ID `0x340 + Node ID`
```

Each packet has a one-byte transport header:

```text
Start packet:    A0 LEN RAW[0..5]       (3..8 CAN data bytes)
Continue packet: B1..BF RAW[next..next+6] (2..8 CAN data bytes)
```

`LEN` is the complete unchanged Step42 binary frame length, including its
`A5 5A` header and CRC. A message up to 78 bytes needs at most 12 CAN frames.
Packets must arrive in order within 30 ms. The original Step42 CRC remains the
end-to-end integrity check. `SDK/step42_can_transport.c/.h` performs this
packing and unpacking for STM32 HAL users.

The default Node ID is `1`, so a new board receives at `0x301` and replies at
`0x341`. Node ID `1~31` is stored in Flash with command `0x21`; therefore
multiple Step42 boards can share one CAN bus. When changing an ID over CAN,
the acknowledgement is sent with the **new** response ID. The PC upper
computer changes it through Debug UART and reads it back in Query status.

CAN supports every binary control command, including position, speed, home,
program and tracking PID commands. Send the same `0x27` signal/error frame
through the CAN segmentation transport. In CAN mode, USART3 Debug remains available to the existing
upper-computer program for Query, telemetry stream, and tracking-PID tuning.
It uses the existing restricted debug permissions and cannot issue external
position, speed, step or home motion commands; CAN remains the sole full
motion-control transport.

Protocol v14 adds this CAN transport without changing the command payloads,
responses, sequence numbers, or CRC.

Protocol v15 adds the Flash-persistent CAN Node ID and appends it as Query
response byte `68`.

Protocol v19 replaces all legacy visual-error inputs with the fixed `0x27`
five-byte payload. The ASCII newline input and command `0x13` are removed.

Protocol v9 extends home mode: `0=nearest single-turn zero`, `1=positive
single-turn zero`, `2=negative single-turn zero`, `3=multi-turn absolute zero`.
Modes `1` and `2` do not use a physical limit switch; they only select which
direction to reach the next `0°` point in the current mechanical revolution.
Mode `3` moves the continuous closed-position coordinate back to `0°`.

Protocol v10 appends `multi_turn_angle_cdeg(i32)` to the Query response at
bytes `62..65`, and to stream telemetry at bytes `27..30`. This is the real
continuous encoder position accumulated by the firmware while it remains
powered. It is independent of whether the position PID is active, so it must
be used for plotting movement in speed mode or while the motor is disabled.

Stream telemetry is: `status, zero_angle_cdeg, target_cdeg(i32),
position_error_cdeg(i32), position_pid_active, rpm_x10, raw_angle,
tracking_primary_centi(i32), tracking_secondary_centi(i32), tracking_output(i16),
tracking_mode, multi_turn_angle_cdeg(i32)`. The Error waveform uses the real
multi-turn position;
the Debug waveform uses the tracking primary error against a fixed zero target.

Examples:

- Enable motor: `CMD=0x02`, payload `01`.
- Set closed-loop position mode: `CMD=0x08`, payload `01`.
- Move to 90.00 deg: `CMD=0x03`, payload `28 23 00 00` because `9000 = 0x00002328`.
- Move to -720.00 deg: `CMD=0x03`, payload is signed little-endian `i32(-72000)`.
- Set open-loop speed mode: `CMD=0x08`, payload `02`.
- Run speed 1000 with accel 3000: `CMD=0x04`, payload `E8 03 B8 0B`.
- Set PID to `Kp=180, Ki=0.2, Kd=0, limit=5000, deadband=0.08`:
  payload `50 46 14 00 00 00 88 13 50 00`.
