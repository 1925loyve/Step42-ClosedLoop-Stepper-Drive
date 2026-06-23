# UART Control Protocol

USART2: 115200, 8 data bits, no parity, 1 stop bit.

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

- `LEN`: payload byte count, maximum 24.
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
| `0x03` | Absolute angle | `u16 angle_cdeg` | `status` |
| `0x04` | Open-loop speed | `i16 speed, u16 accel` | `status` |
| `0x05` | PID | `i16 kp_x100, i16 ki_x100, i16 kd_x100, u16 limit, u16 dead_x1000` | `status` |
| `0x06` | Stream | `u8 enable` | `status` |
| `0x07` | Query | none | `status, enabled, stream, mode, u16 raw, u16 angle_cdeg, i16 pid_output` |
| `0x08` | Mode | `u8 mode` | `status` |
| `0x09` | Stop | none | `status` |

Control modes:

```text
0x00 open-loop position
0x01 closed-loop position
0x02 open-loop speed
0x03 closed-loop speed
```

Examples:

- Enable motor: `CMD=0x02`, payload `01`.
- Set closed-loop position mode: `CMD=0x08`, payload `01`.
- Move to 90.00 deg: `CMD=0x03`, payload `28 23` because `9000 = 0x2328`.
- Set open-loop speed mode: `CMD=0x08`, payload `02`.
- Run speed 1000 with accel 3000: `CMD=0x04`, payload `E8 03 B8 0B`.
- Set PID to `Kp=180, Ki=0.2, Kd=0, limit=5000, deadband=0.08`:
  payload `50 46 14 00 00 00 88 13 50 00`.
