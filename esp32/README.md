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
- Feed-forward plus PI uses `Kp=6`, `Ki=6`, an 11 rad/s feed-forward scale, and a
  10 rad/s target ceiling. Targets are rounded to 0.1 rad/s.
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
