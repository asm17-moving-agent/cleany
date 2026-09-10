# ESP32-S3 motor control, Wi-Fi odometry and hardware tests

`origin/feat/motor-pid-control`의 `6c0fa41`을 기준으로 펌웨어, USB protocol,
제어/통신 단위 테스트, 모터/IMU 하드웨어 테스트, PCB 및 라이브러리 파일을
모두 가져왔다. 이 브랜치의 가감속, 반전 보호, PI 제어, 보정 명령과 진단 화면을
유지하면서 AP+STA 연결과 HTTP encoder snapshot metadata를 추가했다.
Jetson의 기존 Wi-Fi odometry 경로는 그대로 사용할 수 있다.

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

## Motor web interface

The normal firmware creates the `Cleany` Wi-Fi access point with password
`ASM_2026`. Connect to it and open `http://192.168.4.1/`. The left side of the
page has a proportional movement joystick, with a counterclockwise icon button
on its left and a clockwise icon button on its right. Drag the joystick for
forward/backward, sideways or diagonal movement using X-configuration mecanum
mixing. Joystick distance controls the requested speed up to the speed slider's
limit, with a 10% center dead zone. Hold either rotation icon for rotation;
the buttons also support holding Space or Enter. One pointer or keyboard
gesture owns the drive controls at a time. Additional pointers cannot take over
an active gesture. The page also has
signed target-speed controls for each motor. The right side graphs requested,
commanded and measured wheel velocity, with counts and PWM in the table.
The board layout is:

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
Releasing the joystick or rotation icon uses this controlled stop. The joystick
returns to center on release, pointer cancellation, lost capture or window blur.
`STOP ALL` is directly below the drive speed control. `Escape`, a hidden page,
and `pagehide` also stop control. The `STOP ALL` button,
command watchdog, and Wi-Fi disconnect retain an immediate electrical brake for
fail-safe operation. GPIO assignments come from [`PINMAP.md`](PINMAP.md).

Joystick movement updates are coalesced every 50 ms; the held-input heartbeat
remains 250 ms. Control HTTP requests are serialized, and pending commands are
coalesced by control so fast dragging cannot create a backlog. Release/STOP
removes unsent movement commands; a request already in flight finishes before
the stop request is sent. HTTP failure or the 500 ms request timeout clears
held inputs and pending commands, requiring a new gesture before resuming.
The firmware's 750 ms command watchdog remains the fallback on connection loss.
These browser changes do not alter motor PI gains, acceleration, reversal
protection, encoder metadata or odometry interfaces.

Run the browser-control regression tests without hardware (Node.js 18+):

```bash
node --test motor_controller/tests/web_interface_test.cjs
```

These execute the actual inline page script against a fake DOM, clock and HTTP
transport. They cover direction/sign mapping, proportional movement, dead zone,
pointer ownership, cancellation, heartbeat, slow requests, STOP ordering and
connection failure. They do not replace a physical browser or motor test.

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


## 엔코더 Wi-Fi 수신 연결

HTTP의 `encoders`는 원본 PCB 순서 FL/FR/RR/RL과 원시 부호를 유지한다.
PI 피드백과 USB binary telemetry의 논리 방향 보정은 펌웨어 내부에서 적용하고,
HTTP 기반 odom의 방향 보정은 Jetson에서 한 번만 적용한다.

## Wi-Fi 설정

설정하지 않으면 기존 `Cleany` AP와 `192.168.4.1` 웹페이지를 제공한다.
아래 명령으로 station 접속 정보를 생성하면 AP를 유지하면서 같은 Wi-Fi radio로
사무실 AP에도 연결한다. Jetson은 사무실 Wi-Fi와 기존 SSH 연결을 유지할 수 있다.
사무실 AP가 client isolation을 적용하면 같은 SSID에서도 Jetson과 ESP32 간
통신이 막힐 수 있으므로 실제 HTTP 연결을 확인해야 한다.

```bash
python3 motor_controller/scripts/configure_wifi.py --ssid ASM_BUSAN_18F
uvx --with pip --from platformio pio run -d motor_controller
```

비밀번호는 숨김 입력으로 받으며 `src/wifi_credentials.local.h`에만 저장한다.
이 파일과 `.pio/` 빌드 결과는 Git에서 제외한다. 빌드된 펌웨어에는 접속 정보가
포함되므로 이미지 파일을 공유하지 않는다. 자동화는 `--password-stdin`을 지원한다.
로컬 헤더 없이 다시 빌드하면 AP 전용 모드로 돌아간다.

USB 포트를 확인한 뒤 업로드하고 serial log에서 station DHCP 주소를 읽는다.
보드는 기존 설정과 같은 ESP32-S3-DevKitC-1-N32R16V, ESP-IDF 대상이다.

```bash
uvx --with pip --from platformio pio run -d motor_controller --target upload \
  --upload-port /dev/serial/by-id/<ESP32-device>
uvx --with pip --from platformio pio device monitor -d motor_controller \
  --port /dev/serial/by-id/<ESP32-device> --baud 115200
```

성공하면 `Station status endpoint: http://<DHCP-IP>/api/status`를 출력한다.
hostname `cleany-encoder`는 DHCP hostname이며 mDNS 서비스를 추가하지는 않는다.
STA 연결이 끊기면 모터를 정지시키고 다시 연결한다. AP client 연결 해제 시의
기존 전체 정지 동작도 유지한다. 이 펌웨어의 HTTP 모터 제어 페이지가 station
인터페이스에서도 열리므로 접속 가능한 사무실/실험 네트워크에서 사용한다.

## 틱 수신

`GET /api/status`는 기존 제어 진단 필드를 유지하고 아래 MCU metadata를 함께
반환한다. 다음 예시는 encoder 수신에 필요한 필드만 표시한다.

```json
{"encoders":[0,0,0,0],"protocol_version":1,"boot_id":"0123456789abcdef","sample_seq":0,"sample_time_us":123456}
```

순서는
전방 왼쪽, 전방 오른쪽, 후방 오른쪽, 후방 왼쪽이다. 모터 PWM polarity와
encoder count의 부호는 별개이며 tick 값에는 방향 보정을 적용하지 않는다.
상태 요청은 모터 command watchdog을 갱신하거나 모터를 구동하지 않는다.

Jetson에서 [`cleany_base_odometry`](../ros2_ws/src/cleany_base_odometry/README.md)의
`encoder_http_node`를 station IP로 실행하면 `/wheel/encoder_ticks`를 받는다.
`boot_id`는 Wi-Fi 시작 후 한 번 생성한 64-bit random token의 16자리 hex 문자열이다.
`sample_seq`는 상태 snapshot마다 증가하는 uint32 값이고, `sample_time_us`는
`esp_timer_get_time()`으로 읽은 MCU monotonic 시각이다. 카운터와 시각은 같은
critical section에서 읽는다. ISR의 카운터는 unsigned modulo-2^32로 누적하고
응답에는 동일 비트 패턴의 signed int32 값을 사용한다.

Jetson의 `hardware_odometry.launch.py`는 이 metadata로 재시작/누락을 처리한 뒤
wheel odometry와 TF를 발행한다. 절대 시계 동기화나 SLAM 설정까지 포함하지 않는다.
Encoder 부호와 3172 count/rev의 근거는 `feat/motor-pid-control`의 `6c0fa41`이며,
이 브랜치의 HTTP 응답은 부호 보정 전 원본을 유지한다. USB binary protocol의
32-bit boot ID와 HTTP snapshot의 64-bit hex boot ID는 별도 transport 식별자다.
`sample_time_us`는 원시 encoder snapshot 시각이며 PI 진단 필드의 정확한 동시
샘플링을 의미하지 않는다. HTTP 요청과 USB `STATUS`는 모터 watchdog을 갱신하지 않는다.

### 빌드 설정

5 ms 제어 task를 위해 `sdkconfig.defaults`의 `CONFIG_FREERTOS_HZ=1000`을
사용한다. 이전 SDK 설정이 캐시에 남으면 기본값 변경이 반영되지 않을 수 있어
펌웨어에서 tick rate를 compile-time 검사한다. 이 경우 기존 생성물을 보존하고
별도 빌드 디렉터리에서 새 SDK 설정으로 빌드한다.

### 2026-09-10 복원 확인

- 원본 브랜치의 30개 파일을 모두 복원했다. 제어/통신 header와 기존 테스트는
  원본과 동일하고, 네 파일에만 Wi-Fi/HTTP 통합 및 문서/빌드 의존 변경이 있다.
- 가감속, PI, binary codec의 host 테스트 3종과 Jetson ROS 테스트 51개가 통과했다.
- 일반 펌웨어와 모터/IMU 테스트 앱이 빌드됐다. 하드웨어 테스트 앱은 업로드하거나
  실행하지 않았으며, 비영점 모터 명령도 보내지 않았다.
- ESP32에 일반 펌웨어를 업로드하고 HTTP 제어 진단 필드, 웹 속도 그래프 코드,
  USB `HELP`/`STATUS` 응답 및 네 모터의 목표/출력 0을 확인했다.
- 펌웨어 업로드 전후 odom/TF 525개가 일치했고 기존 pose가 유지됐다.
  USB 포트를 여는 과정에서 `USB_UART_CHIP_RESET`이 한 번 관측됐으며,
  이 재시작 후에도 수신이 자동 복구되고 odom/TF 42개가 일치했다.
- 실제 부하 주행, 가감속 시간 실측, USB binary 명령의 실물 구동은 수행하지 않았다.
  IMU는 원본처럼 독립 하드웨어 테스트와 예약 protocol만 제공한다.

로그와 원본/통합 파일 비교는 로컬 `artifacts/motor-control-restore-20260910/`에
보존한다. 펌웨어 이미지에는 Wi-Fi 접속 정보가 포함되므로 Git에서 제외한다.

### 2026-09-11 조이스틱 적용 확인

- 비례 이동 조이스틱과 양옆 반시계/시계 회전 아이콘을 구현했다. Firefox와 niri
  캡처로 desktop/360px 배치를 확인했고, 모의 입력/통신 regression test 11개가
  통과했다. UI raw string 외 펌웨어 코드는 변경 전과 동일하다.
- 기존 실물 app을 백업한 뒤 app 영역만 업로드하고 flash hash를 확인했다.
  첫 reset 후 무응답은 DTR 해제 및 USB hard reset 후 정상 부팅으로 복구됐다.
- 실물 HTTP 페이지가 빌드 소스와 byte-for-byte 일치하고 target/applied 0을
  확인했다. 재실행한 odom은 정지 상태 30초간 약 17.82 Hz, odom/TF 각 534개가
  일치했다. 실제 조이스틱 주행과 모바일 터치 검증은 수행하지 않았다.
- 원자료는 `artifacts/motor-joystick-20260911/upload/`에 보존한다.

## 확인 순서

1. 모터 전원을 끄거나 로봇의 이동을 막은 상태에서 USB 장치와 보드 사양 확인.
2. 펌웨어 빌드, USB 업로드, station IP와 Jetson HTTP 접근 확인.
3. ROS 원본 틱의 수신 빈도와 네 바퀴 값을 확인.
4. 바퀴를 수동으로 돌려 값 변화와 방향을 확인한 뒤 1회전 tick 수 측정.

빌드 성공만으로 실물 연결이나 encoder 변화가 검증된 것은 아니다.
