# ESP32-S3 micro-ROS motor controller

ESP-IDF firmware controls four wheels through native USB micro-ROS, encoder
feedback, and PWM/DIR outputs. The PlatformIO environment is
`esp32-s3-microros`, targeting ESP32-S3 N32R16V with 32 MB octal flash and
16 MB octal PSRAM. GPIO assignments are in [`PINMAP.md`](PINMAP.md).

## Layout

| Path | Contents |
|---|---|
| `src/` | Firmware and motor-control core |
| `host_tests/` | Device-free C++ tests |
| `microros_smoke/` | micro-ROS/type-support link check |
| `pcb/` | KiCad project and project-local libraries |
| `micro_ros/`, `.venv/` | Generated dependencies and build tools |

## Runtime

```text
base/wheel_command → USB XRCE-DDS / rclc callback → latest-value mailbox
                  → motor task: ENABLE / deadline / PI + feed-forward / PWM
encoders → motor task → base/wheel_state → ROS base driver / odometry
```

The communication task owns the USB stream, ROS entities, and Agent reconnect
lifecycle. The independent 5 ms motor task validates commands and updates PWM
under `motorMutex`.

- Startup is PWM zero and disabled. Zero `ENABLE` permits `VELOCITY`.
- Commands validate protocol 2, modular sequence, finite/range, and MCU deadline.
  The sequence high-water survives STOP and reconnect. Rejected commands and
  repeated ENABLE packets do not renew the deadline or reset a moving target.
- STOP has a priority latch and discards queued commands. Its forward sequence
  blocks older delayed ENABLE packets.
- A single deadline, at most 250 ms from receipt, is checked before command
  processing. Expiry and disconnect disable output; resuming needs a new ENABLE.
- Feed-forward plus PI has separate FL/FR/RL/RR settings in `src/main.cpp`:
  `kFlControllerConfig`, `kFrControllerConfig`, `kRlControllerConfig`, and
  `kRrControllerConfig`. Current ground-loaded settings are listed below.
  The aggregate field order is
  **FF scale, Kp, Ki, PWM limit**; FF is `target / FF scale * PWM limit`.
  Edit the relevant wheel's setting, rebuild, and upload to apply it.
  The command target ceiling remains 10 rad/s, with targets rounded to 0.1 rad/s.
- PWM runs at 20 kHz and changes by at most 1 percentage point per 5 ms tick.
  Reversal waits for measured speed at or below 0.5 rad/s for 50 ms before
  changing DIR. STOP/expiry/disconnect reset the controllers and request PWM zero.

Wire arrays use **FL, FR, RL, RR**. PCB motors are M1 FL, M2 FR, M3 RR, M4 RL;
the mapping is `{0,1,3,2}`. Motor and encoder polarity are applied on the MCU.
The encoder scale is 3172 quadrature counts per output revolution.

`base/wheel_command` and `base/wheel_state` use fixed-size
[`cleany_base_interfaces`](../ros2_ws/src/cleany_base_interfaces/README.md)
messages with best-effort, volatile, keep-last-1 QoS. State publication targets
50 Hz. MCU microseconds are monotonic; boot ID identifies encoder baselines
across reboots. Use protocol 2 interfaces and firmware from the same revision.

USB carries framed XRCE-DDS traffic exclusively for the Agent. Agent reachability
is checked every 100 ms with a 20 ms ping timeout. Failure latches STOP before
entity teardown and reconnect.

## 휠별 초기 PI + FF 보정값

2026-10-02, 네 바퀴를 공중에 띄운 상태에서 **네 바퀴를 동시에** 정·역방향으로
PWM 0→100% 스윕했다. PI를 끄고 5% 간격으로 각 단계 1.5초 동안 측정한
실제 PWM·encoder 속도로 FF와 PI 초기값을 추정했다.

| 휠 | FF scale (rad/s) | Kp | Ki | PWM 상한 (%) |
|---|---:|---:|---:|---:|
| FL | 12.4261 | 4.655 | 9.309 | 100 |
| FR | 12.1178 | 4.711 | 9.421 | 100 |
| RL | 11.6403 | 4.864 | 9.728 | 100 |
| RR | 11.8128 | 4.783 | 9.566 | 100 |

FF는 기존 단일 기울기 식을 유지한다. 전 구간의 `PWM ≈ k × 속도` 적합으로
`FF scale = 100/k`를 계산하며, 마찰에 따른 잔여 오차는 PI가 보정한다.
PI 초기값은 PWM→속도 모델의 gain `K`, time constant `τ`, delay `L`에서
`Kp = clamp(0.6/K, 2, 8)`, `Ki = Kp / max(0.5, 4(τ+L))`로 계산했다.
이 모델의 시간 응답에는 기존 50 ms 속도 필터가 포함된다.

보정값을 업로드한 뒤 네 바퀴 동시 정·역방향 **0.5, 2, 5, 10 rad/s** 명령을
각각 5초 동안 검증했다. 마지막 0.5초 평균 속도 오차는 모든 휠에서 4.5% 이내였다.
최종 상태는 구동 비허가, PWM 0, fault 0이다.

### 2026-10-13 지면 하중 실측 캘리브레이션 및 필터 고도화

현재 `src/main.cpp`는 전후이동과 횡이동의 지면 하중 시험에서 도출한 다음
휠별 설정을 사용한다. 진단 기준값은
[`controller_baseline.json`](../tools/wheel_calibration/controller_baseline.json)에
같은 값으로 보존하며, 관련 테스트가 펌웨어 설정과의 일치를 확인한다.

| 휠 | FF scale (rad/s) | Kp | Ki | PWM 상한 (%) | 주요 지면 보정 특성 |
|---|---:|---:|---:|---:|---|
| FL | 11.9352 | 6.058 | 12.118 | 100 | 전후/횡이동 균형 최적화 |
| FR | 12.2596 | 6.030 | 12.058 | 100 | 고효율 FR 휠 출력 과다 방지 |
| RL | 11.7678 | 6.246 | 12.492 | 100 | 후륜 지지 강성 보강 |
| RR | 11.3872 | 6.154 | 12.309 | 100 | 기어박스 마찰 하중 보상 (Yaw 편주 억제) |

명령 슬루는 가속 시 20 ms당 1%, 감속 시 20 ms당 3%로 설정한다.
속도 피드백 LPF 시상수는 `kVelocityFilterTimeConstantSeconds = 0.02`초다.
STOP, 만료 및 연결 단절은 슬루 감속을 기다리지 않고 PWM zero를 요청한다.

측정과 재검증 방법은 [휠 진단 안내](../tools/wheel_calibration/README.md)와
[오도메트리 평가 안내](../tools/odometry_evaluation/README.md)를 따른다.
오도메트리 내부 제어 잔차와 바닥 테이프 또는 LiDAR 기준의 실측 오차는
별도 지표로 기록한다. 측정 JSON과 PNG는 Git 제외 경로인 `artifacts/`에 저장한다.

## Build and device-free verification

From the repository root:

```bash
make firmware-setup
make test-micro-ros-setup test-motor-core
make firmware-smoke
make firmware-build
make build-base test-base
make micro-ros-agent-build
```

Make uses Ubuntu 22.04 `ros2-humble` Distrobox when invoked from the host.
Sources and tool versions are pinned in `micro_ros.lock.json`. Interfaces are
generated from `ros2_ws/src/cleany_base_interfaces`.

Smoke and runtime share a library cache and build sequentially. Changes to
interfaces, metadata, or build environment regenerate type support.
`sdkconfig.defaults` configures octal memory, a 1000 Hz FreeRTOS tick, and the
USB transport. The DOUT image header lets the bootloader enable OPI.

For clangd, run inside Distrobox:

```bash
esp32/.venv/bin/platformio run -d esp32 -e esp32-s3-microros -t compiledb
```

## Upload and communication

Select the device and confirm upload:

```bash
make firmware-upload CLEANY_ESP_PORT=/dev/serial/by-id/<device> CONFIRM_UPLOAD=1
```

Apply the ROS and Agent overlays and use DDS domain 0:

```bash
source /opt/ros/humble/setup.bash
source esp32/micro_ros/agent/install/local_setup.bash
source ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent serial --dev "$CLEANY_ESP_PORT" -b 115200
# Another terminal with the same overlays/domain:
ros2 topic echo /base/wheel_state --qos-reliability best_effort
```

On a bare MCU, check a nonzero boot ID, increasing sequence/timestamps,
`enabled=false`, zero targets/PWM, and no fault.

## Robot acceptance

Use a supervised, people-free area, reviewed hardware limits, and an accessible
physical emergency stop.

1. Lift the wheels; check startup zero PWM, wheel directions, and encoder signs.
2. Enable and send a new low-speed command; check forward/left/yaw patterns,
   PI response, PWM slew, and encoder-confirmed reversal dwell.
3. Measure STOP, command expiry, driver/Agent exit, USB loss, and MCU reboot
   behavior. Reconnect requires a new enable and command.
4. Verify motion, odometry, and single odom/TF ownership with
   [`cleany_base_driver`](../ros2_ws/src/cleany_base_driver/README.md).

Record revisions, settings, load/surface, wheel signs, rates, and stop times.

## KiCad project

[`pcb/motor_controller.kicad_pro`](pcb/motor_controller.kicad_pro) uses project-local
symbol and footprint tables. The Espressif assets and license are documented in
[`pcb/libraries/README.md`](pcb/libraries/README.md).
