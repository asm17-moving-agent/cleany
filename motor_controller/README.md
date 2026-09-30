# ESP32-S3 motor and IMU hardware tests

Build/test commands in this document run inside Ubuntu 22.04 `ros2-humble`.
Root Make targets enter that Distrobox automatically when invoked from the host.

The normal PlatformIO firmware entry point is `src/main.cpp`. Hardware checks
are independent Unity applications under `test/`, so a test runs only when it
is explicitly selected with `pio test`.

The PlatformIO environment targets the ESP32-S3-DevKitC-1-N32R16V with 32 MB
octal flash and 16 MB octal PSRAM. Firmware and hardware tests use ESP-IDF;
the Arduino framework is not required.

The environment uses PlatformIO's official `esp32-s3-devkitc-1` board
template. `platformio.ini` and `sdkconfig.defaults` supply the N32R16V memory
overrides. The `dout` image-header mode is intentional: the ESP32-S3
bootloader switches the detected octal flash to OPI mode.

## micro-ROS build integration

The `esp32-s3-microros` environment uses the official Humble ESP-IDF component
with custom XRCE-DDS stream transport. The default PlatformIO environment
remains `esp32-s3-devkitc-1-n32r16v` (commissioning and COBS). The separate
`microros_smoke/` project links `rclc` and both Cleany message type-support
entrypoints without starting motors or opening a transport.

From the repository root:

```bash
make firmware-setup
make test-micro-ros-setup test-motor-core
make firmware-smoke
make firmware-build
make firmware-commissioning
make micro-ros-agent-build
```

These commands run inside Ubuntu 22.04 `ros2-humble`; Make enters that
Distrobox when called from the host. Sources and Python tool versions are
pinned in `micro_ros.lock.json` and `tools/micro_ros_setup.py`. The official
component's branch-based source clones are resolved to reviewed commit IDs by
the bootstrap Git wrapper. `make firmware-setup` creates ignored external
checkouts and links the single message package source from
`ros2_ws/src/cleany_base_interfaces`.

Message, metadata, sdkconfig and project/environment changes invalidate
generated type support. Changing `sdkconfig.microros.defaults` regenerates the
derived sdkconfig; keep reviewed settings in that source file. Smoke and
runtime builds share one component library cache and must run sequentially.
Use `python3 tools/micro_ros_setup.py --check` inside Distrobox to verify
external source revisions. Setup and build commands do not upload firmware.

### Runtime and command gate

`make firmware-build` builds `esp32-s3-microros`: the native USB transport,
`rclc` subscription/publisher and reconnect loop run in their own task. No web,
Wi-Fi AP, text CLI or COBS receiver starts in this build. The commissioning
environment retains those entrypoints.

The subscription copies fixed-size commands to a one-item overwrite mailbox.
STOP has a separate pending latch so it cannot be overwritten by velocity.
The existing 5 ms motor-control task consumes the mailbox under `motorMutex`,
checks boot/session/sequence, finite/range and MCU deadlines, then runs the
existing feed-forward + PI and final PWM slew/reversal protection.
Commands use logical FL, FR, RL, RR order, mapped to PCB indices `{0,1,3,2}`.
The existing integer-percent target path rounds wheel goals to 0.1 rad/s.

Only fresh accepted zero ARM or armed VELOCITY refreshes the 250 ms watchdog.
The same control task checks active-command expiry before each PWM update;
the earlier of the deadline and watchdog stops/disarms the motors. Rejected
packets never refresh either boundary. STOP and reconnect discard prior goals;
motion requires a new zero session, explicit ARM and a fresh velocity command.
Recently retired sessions remain blocked while queued packets can be valid.
The bounded eight-session history fails closed with a 250 ms handshake holdoff
if rapid session churn exhausts it.

Agent reachability is checked every 100 ms with a 20 ms ping timeout. Failure
latches STOP before bounded entity teardown and a fresh connection attempt.
State publication targets 50 Hz; device tests must measure actual USB timing.
`make test-motor-core` covers session/deadline replay, watchdog, encoder rollover,
wheel mapping and bounded USB I/O without opening a device.

Upload is a separate, explicitly confirmed operation after hardware preparation:

```bash
make firmware-upload CLEANY_ESP_PORT=/dev/serial/by-id/<confirmed-device> \
  CONFIRM_UPLOAD=1
```

### Native USB stream

`src/usb_stream_transport.hpp` separates bounded stream I/O from the ESP-IDF
device port. The custom transport uses XRCE-DDS framing (`framing=true`), not
COBS. Reads preserve partial data and return zero on timeout; writes complete
the requested frame bytes within their deadline or report failure. Closed
transport use and I/O errors are rejected. The ESP-IDF port uses native
`usb_serial_jtag_read_bytes` and `usb_serial_jtag_write_bytes`.

The micro-ROS sdkconfig disables application/bootloader console output on USB.
Do not attach a text monitor or send commissioning commands to the Agent's
stream. The legacy environment retains its own commissioning console and COBS
decoder. Host test doubles exercise open/close, partial I/O, timeout and error
paths with `make test-motor-core`.

## Motor web interface

The commissioning firmware creates the `Cleany` Wi-Fi access point with password
`ASM_2026`. Connect to it and open `http://192.168.4.1/`. The left side of the
page has press-and-hold controls for forward, backward, left, right, clockwise,
and counterclockwise motion using X-configuration mecanum mixing. It also has
signed PWM controls for each motor. The right side graphs live quadrature
counts from encoders 1–4. The board layout is:

```text
          FRONT
    M1 FL       M2 FR
    M4 RL       M3 RR
```

Motor numbering proceeds clockwise from M1 at front-left. M1 and M4 direction
polarity is inverted in firmware to match the installed wheel orientation;
M2 and M3 use normal polarity. Encoder polarity is calibrated separately so
positive measured velocity matches a positive logical wheel command. Verify
all wheel directions and measured velocity signs with the robot lifted before
driving it. Each active motor must receive the browser's 250 ms heartbeat and
is stopped by the firmware after 750 ms without a command.
Motor commands pass through a 5 ms, 1%-per-tick slew-rate limiter, taking about
500 ms from 0 to 100%. This limiter is applied to the final PI-controlled PWM
output, so feedback correction cannot bypass the motor-protection rate limit.
A direction change ramps to zero and waits until
encoder feedback remains below 0.5 rad/s for 50 ms before changing `DIR`.
Releasing a drive button uses this controlled stop. The `STOP ALL` button,
command watchdog, and Wi-Fi disconnect retain an immediate electrical brake for
fail-safe operation. GPIO assignments come from [`PINMAP.md`](PINMAP.md).

Each ramped percentage command is mapped to a target output-shaft velocity,
where 100% currently means 10 rad/s. A per-wheel feed-forward plus PI loop
uses an 11 rad/s measured no-load feed-forward scale and encoder feedback to
produce the applied PWM percentage. The initial
controller values (`Kp=6`, `Ki=6`) are hardware response-calibrated baseline
gains, not a substitute for loaded-floor validation. Verify encoder polarity
with the robot lifted before loaded driving.

The web debug panel displays a selectable motor's requested, rate-limited, and
measured velocity response over the latest 20 seconds. Its table also shows
encoder counts, tracking error, final PWM, and controller state for every
motor.
The same values are available from `/api/status` as `encoders`, `target`,
`applied`, `target_rad_s`, `commanded_rad_s`, `omega_rad_s`, and
`reverse_waiting`. Velocity conversion uses 3172 quadrature counts per output
revolution (13 PPR, 4x decoding, 61:1 reduction).

## USB serial control and calibration

The USB Serial/JTAG port provides motor control and telemetry without changing
the host's network connection. The COBS-framed, CRC-protected binary Jetson
interface, including wheel telemetry and the reserved MPU6050 frame, is specified in
[`SERIAL_PROTOCOL.md`](SERIAL_PROTOCOL.md). The following newline-terminated
commands remain available for manual commissioning only:

```text
HELP
STATUS
MOTOR <1-4> <-100..100 percent>
VELOCITY <1-4> <-10..10 rad/s>
MOVE <1-4> <-10..10 rad/s> <0.5..6 rad>
MOVE ALL <-10..10 rad/s> <0.5..6 rad>
TRACE <1-4> <20..1000 ms>
TRACE ALL <50..1000 ms>
TRACE STOP
STOP
```

`MOVE` is intended for response calibration. `MOVE ALL` drives all wheels in
the same logical direction to avoid the wheel slip caused by running one wheel
against three stationary wheels. Each wheel commands an immediate electrical
stop at its encoder threshold or after five seconds. Firmware places that
threshold 0.25 rad before the requested limit to reserve room for control-loop
and mechanical stopping latency; physical coasting still depends on load and
surface and is not a position-control guarantee. `TRACE` emits CSV-like
`CLEANY_TRACE` records containing timestamp, motor, encoder count, requested
velocity, rate-limited velocity, measured velocity, PWM, and bounded-move
state. `STOP` remains an immediate stop.

Firmware boots in manual commissioning mode. A binary frame's leading NUL byte
switches the serial receiver into binary mode until reboot, preventing binary
payload bytes from being misinterpreted as manual commands.

Build and upload the non-test firmware with PlatformIO isolated by `uv`:

```bash
uvx --with pip --from platformio pio run -d motor_controller
uvx --with pip --from platformio pio run -d motor_controller --target upload \
  --upload-port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:E5:B1:D5:3F:34-if00
uvx --with pip --from platformio pio device monitor -d motor_controller \
  --port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:E5:B1:D5:3F:34-if00 \
  --baud 115200
```

## Editor tooling

Generate the clangd compilation database after dependencies or build flags
change:

```bash
pio run -e esp32-s3-devkitc-1-n32r16v -t compiledb
```

The project-local `.clangd` configuration and compilation database provide
ESP32-S3 completion and diagnostics without editor-specific configuration.

Run the host-side command-filter check without attached hardware:

```bash
c++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -Imotor_controller/src \
  motor_controller/tests/motor_command_filter_test.cpp \
  -o /tmp/motor_command_filter_test &&
  /tmp/motor_command_filter_test
```

Run the host-side velocity-controller check:

```bash
c++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -Imotor_controller/src \
  motor_controller/tests/wheel_velocity_controller_test.cpp \
  -o /tmp/wheel_velocity_controller_test &&
  /tmp/wheel_velocity_controller_test
```

Run the host-side binary protocol codec check:

```bash
c++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -Imotor_controller/src \
  motor_controller/tests/serial_protocol_test.cpp \
  -o /tmp/serial_protocol_test &&
  /tmp/serial_protocol_test
```

## Hardware tests

### micro-ROS 후속 실물 인수 시험

현재 단계의 빌드 및 host/mock 테스트와 아래 실물 검증은 별개다. 펌웨어 upload와
하드웨어 시험은 운영자의 별도 명시 요청 후 진행한다. 로봇과 emergency stop이
준비되고 아래 항목이 통과해야 Task 4의 실물 검증 및 Story 완료를 판정할 수 있다.

1. **준비:** 사람 없는 통제 구역, 구역 밖 감독자, 즉시 사용할 수 있는 물리
   emergency stop, 공통 ground, 배선 및 전원 정격을 확인한다. 휠을 들어 올린다.
2. **MCU-only 통신:** 모터 전원을 분리한 상태에서 native USB 장치의 persistent
   by-id path, Agent 연결, WheelState 50 Hz, boot/session 식별자와 console stream
   분리를 확인한다. 재부팅 시 boot ID 변경과 disarmed 상태를 확인한다.
3. **무허가 정지:** Agent 연결, launch 시작, `/cmd_vel` 단독 발행으로 PWM이
   발생하지 않는지 확인한다. 새 session은 zero/disarmed여야 한다.
4. **올린 상태의 방향:** 명시적 enable 후 새 저속 명령으로 FL, FR, RL, RR 순서와
   기존 logical-forward/encoder 부호를 확인한다. ROS에서 부호를 다시 반전하지 않는다.
   전진, 좌측 병진, 반시계 회전의 wheel pattern을 확인한다.
5. **보호 동작:** 기존 PI + feed-forward, 최종 PWM slew limiter, encoder-confirmed
   reversal dwell을 trace로 확인한다. STOP 및 emergency stop의 실제 전원 차단을
   각각 확인한다. 소프트웨어 STOP/watchdog은 물리 emergency stop을 대체하지 않는다.
6. **고장 주입:** 명령 발행 중지, driver 종료, Agent 종료, USB 분리, MCU 재부팅,
   잘못된 session/sequence/deadline 명령에서 정지 및 watchdog을 확인한다.
   MCU 250 ms timeout과 task 관측 지연, 실제 정지 시간을 각각 측정한다.
7. **재연결:** 이전 이동 목표가 복원되지 않고 새 handshake, 명시적 enable,
   enable 확인 뒤의 새 `/cmd_vel`이 있어야만 구동하는지 확인한다.
8. **바닥 시험:** 실측 geometry와 검토된 낮은 주행 제한을 사용한다. `/joint_states`,
   `/wheel/odom`, `/odom`, `odom -> base_link`의 publisher 소유권을 확인하고 전진,
   횡이동, 회전을 짧게 시험한다. 실제 이동량과 odometry, slip 및 정지 거리를 기록한다.

기록에는 firmware/Agent revision, 설정 파일, 전원과 하중, 표면, wheel sign,
명령/피드백 주기, 고장별 정지 시간과 합격 여부를 포함한다. Odometry 보정과
SLAM/Nav2 자율주행 검증은 이 인수 시험 뒤 별도 작업이다.

Run both suites in sequence with PlatformIO port auto-detection:

```bash
uvx --with pip --from platformio pio test -d motor_controller \
  -e esp32-s3-devkitc-1-n32r16v
```

If the board was left disconnected by an older test build, perform the
BOOT/RESET upload sequence once before running this command. Test builds from
this project keep the USB Serial/JTAG console active between suites, so
subsequent uploads should not require that sequence.

Set the connected board port once:

```bash
CLEANY_ESP_PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:E5:B1:D5:3F:34-if00
```

Run only the motor test:

```bash
uvx --with pip --from platformio pio test -d motor_controller \
  -e esp32-s3-devkitc-1-n32r16v -f test_motor \
  --upload-port "$CLEANY_ESP_PORT" --test-port "$CLEANY_ESP_PORT"
```

The motor test uses encoder A on GPIO4, encoder B on GPIO3, direction on
GPIO10, and PWM on GPIO11. It ramps to 50% duty, runs for at most three
seconds, and ramps down before reporting whether 793 encoder counts were
reached. The motor supply, driver, encoder, and development board must share
ground.

Run only the MPU6050 test:

```bash
uvx --with pip --from platformio pio test -d motor_controller \
  -e esp32-s3-devkitc-1-n32r16v -f test_imu \
  --upload-port "$CLEANY_ESP_PORT" --test-port "$CLEANY_ESP_PORT"
```

Connect MPU6050 VCC to 3V3, GND to GND, SDA to GPIO8, and SCL to GPIO9. The
test accepts address `0x68` or `0x69`, checks `WHO_AM_I`, wakes the device,
and reads a complete accelerometer, temperature, and gyroscope frame.

The project-local `test/unity_config.h` leaves the ESP-IDF USB Serial/JTAG
console active after `UNITY_END()`. This allows PlatformIO to upload the next
test suite without another manual BOOT/RESET sequence.

## KiCad symbol and footprint

The KiCad project is
[`pcb/motor_controller.kicad_pro`](pcb/motor_controller.kicad_pro). Its
project-local library tables register Espressif's official
ESP32-S3-DevKitC symbol and footprint as
`PCM_Espressif:ESP32-S3-DevKitC`. The vendored source revision, mechanical
drawing used for verification, and license are recorded in
[`libraries/README.md`](libraries/README.md).
