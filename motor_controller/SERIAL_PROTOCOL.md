# Cleany ESP32–Jetson binary serial protocol

Status: protocol 1.0, implemented by `src/main.cpp`. This document fixes the
motor-controller transport boundary only. Real mecanum geometry, odometry
fusion, ROS topic names, and MPU6050 mounting calibration remain Jetson-side
deployment decisions.

## Transport and mode selection

- ESP32-S3 native USB Serial/JTAG; Linux must open a configured persistent
  `/dev/serial/by-id/...` path.
- Host libraries use 115200 8-N-1. Native USB does not physically use this baud
  rate.
- Firmware boots in newline-based commissioning mode (`HELP`, `STOP`, `MOVE`,
  and `TRACE`).
- Sending a zero byte enters binary mode until reboot. A binary host naturally
  does this because every frame starts with zero.
- Entering binary mode disables manual trace output. Production software must
  not send text commands on the binary connection.

ESP-IDF logs can still appear outside frames. A host ignores non-COBS segments
and accepts only packets with valid magic, version, length, and CRC. Production
builds should minimize console logs.

## Framing and primitive types

```text
0x00 | COBS(decoded packet) | 0x00
```

Decoded packet:

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint16` | Magic `0x4C43` (wire bytes `43 4C`, ASCII `CL`) |
| 2 | `uint8` | Protocol major, currently 1 |
| 3 | `uint8` | Protocol minor, currently 0 |
| 4 | `uint8` | Message type |
| 5 | `uint8` | Flags |
| 6 | `uint16` | Payload length |
| 8 | `uint32` | Sequence |
| 12 | bytes | Payload |
| final 2 | `uint16` | CRC |

- All integers are little-endian.
- Floating-point fields are little-endian IEEE-754 binary32.
- Maximum decoded packet length is 240 bytes.
- CRC is CRC-16/CCITT-FALSE: polynomial `0x1021`, initial `0xFFFF`, no
  reflection, no final XOR. It covers the header and payload.
- Implementations serialize fields explicitly. They must not transmit native
  C/C++ structs because padding and ABI are not part of this protocol.
- Invalid COBS, CRC, magic, major version, length, non-finite values, or ranges
  do not change motor output or refresh the command watchdog.

Flag bit 0 is `ACK_REQUIRED`. It is normally clear on the 50 Hz wheel-command
stream.

## Wheel convention

Every four-wheel wire field is ordered:

```text
front_left, front_right, rear_left, rear_right
```

This deliberately follows the ROS-facing convention rather than PCB order.
Firmware maps wire indices to M1, M2, M4, M3 because the PCB array is FL, FR,
RR, RL. Positive position and velocity use the firmware-calibrated logical
wheel-forward direction.

- Angular position unit: output-shaft radian.
- Angular velocity unit: output-shaft `rad/s`.
- Encoder position transport: signed counts, 3172 counts/revolution.
- PWM telemetry: signed percent.

The Jetson owns `cmd_vel` mecanum inverse kinematics. Real wheel radius,
wheelbase, and track width are not hidden in firmware.

## Connection sequence

1. Open and flush the serial device.
2. Generate a random, nonzero `session_id`.
3. Send `HELLO_REQUEST`. Handling this request immediately stops all motors and
   replaces any previous control session.
4. Validate the `HELLO` response: protocol, echoed session, capabilities,
   counts/revolution, speed limit, and watchdog.
5. Request telemetry with `STREAM_CONFIG`.
6. Send zero `WHEEL_COMMAND` frames before allowing nonzero ROS commands.
7. Send `STOP` on orderly shutdown. USB loss is independently covered by the
   ESP watchdog.

A new `boot_id` invalidates timestamp offset, stream configuration, session,
and command sequence state.

## Message types

| Value | Direction | Name |
|---:|---|---|
| `0x01` | Jetson → ESP | `HELLO_REQUEST` |
| `0x02` | Jetson → ESP | `STREAM_CONFIG` |
| `0x03` | Jetson → ESP | `TIME_SYNC_REQUEST` |
| `0x10` | Jetson → ESP | `WHEEL_COMMAND` |
| `0x11` | Jetson → ESP | `STOP` |
| `0x81` | ESP → Jetson | `HELLO` |
| `0x82` | ESP → Jetson | `ACK` |
| `0x83` | ESP → Jetson | `TIME_SYNC_RESPONSE` |
| `0x90` | ESP → Jetson | `WHEEL_STATE` |
| `0x91` | ESP → Jetson | `IMU_STATE` (reserved) |

### `HELLO_REQUEST` (`0x01`, 8-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32` | Nonzero host `session_id` |
| 4 | `uint32` | Requested capability bits |

### `HELLO` (`0x81`, 32-byte payload)

The response header sequence equals the request sequence.

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32` | Random ESP `boot_id` |
| 4 | `uint32` | Active host `session_id` |
| 8 | `uint8[3]` | Firmware major, minor, patch |
| 11 | `uint8` | Reserved |
| 12 | `uint32` | Capability bits |
| 16 | `float32` | Maximum wheel target, rad/s |
| 20 | `uint32` | Encoder counts/revolution |
| 24 | `uint16` | Command timeout, ms |
| 26 | `uint16` | Motor control frequency, Hz |
| 28 | `uint16` | Maximum wheel-state rate, Hz |
| 30 | `uint16` | Maximum IMU-state rate, Hz |

Capability bits:

| Bit | Meaning |
|---:|---|
| 0 | Wheel velocity control |
| 1 | IMU available |
| 2 | Time synchronization |

The current firmware advertises bits 0 and 2. It does not advertise IMU until
MPU6050 support is actually implemented.

### `WHEEL_COMMAND` (`0x10`, 20-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32` | Active `session_id` |
| 4 | `float32[4]` | FL, FR, RL, RR target rad/s |

- Each target must be finite and within `[-10, 10]`.
- The four targets are applied atomically.
- Send at 50 Hz.
- Targets are currently quantized to 0.1 rad/s by the preserved 1% command
  rate-limiter representation.
- A valid command sequence must be newer than the previous sequence using
  signed 32-bit modular comparison.
- No ACK is sent unless `ACK_REQUIRED` is set. The processed sequence appears
  in `WHEEL_STATE`.
- If no valid command arrives for 250 ms, all PWM outputs are immediately set
  to zero. The 25 ms watchdog task means observation can occur up to one task
  period later.

### `STOP` (`0x11`, 8-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32` | Session ID |
| 4 | `uint8` | Reason |
| 5 | `uint8[3]` | Reserved |

STOP clears targets, PI state, rate-limit state, calibration moves, and PWM.
A CRC-valid STOP always performs the stop even with a wrong session; its ACK
then reports `BAD_SESSION`.

Suggested reason values are normal shutdown 0, ROS timeout 1, operator stop 2,
driver fault 3, and reconnect 4.

### `STREAM_CONFIG` (`0x02`, 8-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32` | Active session ID |
| 4 | `uint16` | Wheel period in ms |
| 6 | `uint16` | IMU period in ms |

Wheel period is 0 to disable or 20–1000 ms. IMU period currently must be zero;
a nonzero value returns `UNSUPPORTED`. Stream settings are not persisted.

### `TIME_SYNC_REQUEST` (`0x03`, 12-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32` | Active session ID |
| 4 | `uint64` | Host monotonic nanoseconds |

### `TIME_SYNC_RESPONSE` (`0x83`, 24-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint64` | Echoed host monotonic ns |
| 8 | `uint64` | ESP receive monotonic us |
| 16 | `uint64` | ESP transmit monotonic us |

Do not reinterpret ESP monotonic microseconds as ROS epoch time. The Jetson
either estimates a monotonic offset or stamps messages at receipt.

### `ACK` (`0x82`, 4-byte payload)

The response header sequence equals the request sequence.

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint8` | Request message type |
| 1 | `uint8` | Status |
| 2 | `uint16` | Detail, currently zero |

| Status | Value |
|---|---:|
| OK | 0 |
| BAD_PAYLOAD | 1 |
| OUT_OF_RANGE | 2 |
| UNSUPPORTED | 3 |
| BAD_SESSION | 4 |
| STALE_SEQUENCE | 5 |
| BUSY | 6 |
| INTERNAL_ERROR | 7 |

### `WHEEL_STATE` (`0x90`, 96-byte payload)

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint64` | ESP monotonic timestamp, us |
| 8 | `uint32` | Boot ID |
| 12 | `uint32` | Active session ID |
| 16 | `uint32` | Last accepted wheel-command sequence |
| 20 | `uint16` | Command age in ms; 65535 when unavailable |
| 22 | `uint8` | Control mode |
| 23 | `uint8` | Reserved |
| 24 | `int32[4]` | Logical encoder counts |
| 40 | `float32[4]` | Measured rad/s |
| 56 | `float32[4]` | Requested target rad/s |
| 72 | `float32[4]` | Rate-limited command rad/s |
| 88 | `int8[4]` | Applied PWM percent |
| 92 | `uint32` | Fault bits |

Control modes are stopped 0, serial velocity control 1, reverse wait 2,
calibration 3, serial watchdog stop 4, and manual/web control 5.

Fault bit 0 records a serial command timeout. Fault bits are session-sticky and
clear on a valid new `HELLO_REQUEST`.

## Reserved MPU6050 message

`IMU_STATE` (`0x91`) is reserved with a 40-byte payload:

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint64` | ESP monotonic timestamp, us |
| 8 | `float32[3]` | Acceleration, m/s² |
| 20 | `float32[3]` | Angular velocity, rad/s |
| 32 | `float32` | Temperature, °C |
| 36 | `uint32` | Sensor status |

Values remain in the physical sensor frame. Future firmware must not invent an
orientation quaternion. The ROS adapter owns the configured frame ID, mounting
transform, calibrated bias, and covariance. Adding MPU6050 support does not
change wheel packet layouts or protocol major version.
