# UART Control Protocol

Error UART (USART2) and Debug UART (USART3): 115200, 8 data bits, no parity,
1 stop bit. The upper computer identifies which one it is connected to from
the first Query response.

The firmware command interface uses this binary protocol only. The PC tuning
tool and MCU/Linux SDK use the same frame format.

## Frame

Request:

```text
A5 5A LEN CMD SEQ PAYLOAD CRC8
```

Response:

```text
5A A5 LEN CMD|80 SEQ PAYLOAD CRC8
```

- `LEN`: payload byte count, maximum 64.
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
| `0x13` | Tracking error input | `i32 primary_centi, i32 secondary_centi`, each `-100000~100000` | `status` |
| `0x17` | Relative closed-loop angle | `i32 delta_cdeg`, range `-720000~720000`; resulting target must remain within `-7200°~7200°` | `status` |
| `0x18` | Program mode | `u8 enable` | `status`; accepted by Error and Debug UART |
| `0x19` | Program step | `u8 index, u8 op, u8 flags, u16 speed, u16 accel, u16 duration_ms, i32 value` | `status`; accepted by Error and Debug UART |
| `0x1A` | Save program | `u8 count`, maximum 8 | `status`; commits staged steps to Flash through either UART |
| `0x1B` | Run program | none | `status`; requires program mode and motor enabled |
| `0x1C` | Program query | none | `status, program_mode, program_running, program_count` |

Control modes:

```text
0x00 open-loop position
0x01 closed-loop position
0x02 open-loop speed
0x03 built-in error PID tracking
```

Protocol v6 appends `response_interface` to the Query response: `0=Error/USART2`,
`1=Debug/USART3`. The upper computer uses it to choose its waveform meaning
automatically; the host does not need to select an interface type manually.

Protocol v7 further appends `program_mode`, `program_running`, and
`program_count` to Query. While program mode is enabled, external motion
commands (`angle`, `speed`, `relative angle`, `move steps`, `home`, mode change,
and tracking error input) return `DENIED`; stop, telemetry and parameter writes
remain available. The program executor itself may run position, open-step,
speed, delay, home, tracking PID, microstep setup, chopper setup and stop steps
without blocking the control loop. Program step `op` values are: `1` absolute
position, `2` relative position, `3` open-loop steps, `4` speed, `5` delay,
`6` home, `7` tracking PID, `8` stop, `9` microsteps (`value=1/2/.../256`),
and `10` chopper (`flags=0` StealthChop, `1` SpreadCycle).

Stream telemetry is: `status, zero_angle_cdeg, target_cdeg(i32),
position_error_cdeg(i32), position_pid_active, rpm_x10, raw_angle,
tracking_primary_centi(i32), tracking_secondary_centi(i32), tracking_output(i16),
tracking_mode`. The Error waveform uses target/error for multi-turn position;
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
