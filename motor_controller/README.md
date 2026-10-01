# ESP32-S3 micro-ROS motor controller

Firmware contains native USB micro-ROS, encoder feedback and four-wheel motor
control. The only normal PlatformIO environment is `esp32-s3-microros`.
GPIO assignments are documented in [`PINMAP.md`](PINMAP.md).

## Runtime

```text
base/wheel_command → USB XRCE-DDS / rclc callback → latest-value mailbox
                  → motor task: ENABLE / deadline / PI + feed-forward / PWM
encoders → motor task → base/wheel_state → ROS base driver / odometry
```

The communication task owns the native USB Serial/JTAG custom stream,
`rclc` subscription/publisher and Agent reconnect lifecycle. The 5 ms motor
task independently validates and applies commands under `motorMutex`; the
callback does not calculate PI or write PWM.

### Commands and protection

- Startup is zero/disabled. An explicit zero `ENABLE` permits `VELOCITY`;
  session negotiation and command authorization tokens are not used.
- Commands validate protocol v2, modular sequence, finite/range and MCU deadline.
  The sequence high-water survives STOP and reconnect. Rejected commands and
  repeated ENABLE packets do not renew the deadline or reset a moving target.
- STOP has a separate priority latch and discards queued commands. Its forward
  sequence blocks older delayed ENABLE packets.
- A single deadline, bounded to at most 250 ms from receipt, is checked before
  command processing and PWM updates. Expiry disables the controller; later
  velocities cannot re-enable it. Reconnect requires a new explicit ENABLE.
- Feed-forward plus PI retains `Kp=6`, `Ki=6`, 11 rad/s feed-forward scale
  and the 10 rad/s firmware target ceiling. The existing integer-percent
  command path rounds targets to 0.1 rad/s.
- Final PWM changes by at most 1% per 5 ms tick. Reversal ramps to zero and
  requires measured speed below 0.5 rad/s for 50 ms before changing DIR.
  STOP/expiry/disconnect reset the filters/controllers and request PWM zero.

Wire arrays use **FL, FR, RL, RR**; PCB motors are M1 FL, M2 FR, M3 RR, M4 RL.
Both commands and telemetry use the `{0,1,3,2}` mapping. Existing motor direction
and encoder polarity are applied once, on the MCU. The encoder scale is
3172 quadrature counts per output revolution.

### ROS and USB

`base/wheel_command` and `base/wheel_state` use fixed-size
[`cleany_base_interfaces`](../ros2_ws/src/cleany_base_interfaces/README.md)
messages with best-effort, volatile, keep-last-1 QoS. State publication targets
50 Hz. MCU microseconds are monotonic, not ROS epoch time. Protocol v2 changes
both message layouts; rebuild ROS interfaces/driver and firmware together.
Telemetry boot ID identifies encoder baselines across reboots, not command
authority.

`src/usb_stream_transport.hpp` separates bounded stream I/O from the ESP-IDF
port. XRCE-DDS framing is enabled; partial I/O, timeout and error paths have
host tests. Agent reachability is checked every 100 ms with a 20 ms ping
timeout. Failure latches STOP before bounded entity teardown and reconnect.

Application/bootloader console output is disabled. The Agent exclusively owns
the USB stream; do not attach a text monitor. The firmware exposes no Wi-Fi AP,
HTTP server, text command interface, custom serial codec or calibration/trace
command path. IMU support is not part of this runtime.

## Build and device-free verification

From the repository root, inside Ubuntu 22.04 `ros2-humble`:

```bash
make firmware-setup
make test-micro-ros-setup test-motor-core
make firmware-smoke
make firmware-build
make build-base test-base
make micro-ros-agent-build
```

Make enters that Distrobox automatically when invoked from the host. Sources
and tool versions are pinned in `micro_ros.lock.json` and
`tools/micro_ros_setup.py`. Interfaces are generated from the single
`ros2_ws/src/cleany_base_interfaces` source; external sources and generated
files stay ignored.

`sdkconfig.defaults` contains the N32R16V memory settings, 1000 Hz FreeRTOS tick
and USB-only micro-ROS settings. The ESP32-S3 board has 32 MB octal flash and
16 MB octal PSRAM. The DOUT image header is intentional; the bootloader enables
OPI. Changing reviewed defaults regenerates the derived sdkconfig.

`microros_smoke/` links `rclc` and both message type-support entrypoints without
opening USB or initializing motors. Smoke/runtime share a library cache and
run sequentially. Project/environment changes rerun CMake; interface and
metadata changes regenerate type support.

The six C++ tests cover command/PWM protection, PI/feed-forward, modular
encoders, wheel mapping, ENABLE/STOP/deadline and bounded USB transport.
They do not open hardware. ROS graph tests use the isolated synthetic mock.

For clangd, run the pinned PlatformIO executable inside Distrobox:

```bash
motor_controller/.venv/bin/platformio run -d motor_controller \
  -e esp32-s3-microros -t compiledb
```

## Upload and MCU-only communication

Upload is a separate, explicitly confirmed operation:

```bash
make firmware-upload CLEANY_ESP_PORT=/dev/serial/by-id/<confirmed-device> \
  CONFIRM_UPLOAD=1
```

Then apply the ROS and Agent overlays and use DDS domain 0:

```bash
source /opt/ros/humble/setup.bash
source motor_controller/micro_ros/agent/install/local_setup.bash
source ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev "$CLEANY_ESP_PORT" -b 115200
# In another terminal with the same overlays/domain:
ros2 topic echo /base/wheel_state --qos-reliability best_effort
```

Bare-MCU checks require no enable or movement command. Observe a nonzero
boot ID, increasing state sequence/MCU timestamps, `enabled=false`,
zero targets/PWM and no fault. Motor/encoder GPIO configuration does not
require connected external devices.

On 2026-09-30, the pre-cleanup firmware at repository revision `97ae217` was
uploaded and hash-verified on ESP32-S3 rev v0.2 using native USB Serial/JTAG.
With motors, encoders and IMU disconnected, 482 states over 10.007 s measured
48.07 Hz, maximum receive gap 24.95 ms and no sequence loss. All samples were
disarmed with session/fault/PWM zero; no motor commands were sent. The test
Agent was stopped afterward. This record applies to that uploaded firmware,
not automatically to later builds.

## Robot acceptance

Robot checks require an explicit request, a people-free controlled area,
supervision, reviewed hardware limits and an accessible physical emergency
stop. Software STOP/command expiry do not replace it.

1. Lift the wheels; verify startup/disabled/connection-only zero PWM and each
   wheel's logical direction and encoder sign.
2. After explicit enable and a new low-speed command, verify forward/left/yaw
   patterns, PI response, final PWM slew and encoder-confirmed reversal dwell.
3. Measure STOP, command deadline, driver/Agent exit, USB loss and MCU reboot
   behavior. Reconnect must require new enable and command.
4. With confirmed geometry and reviewed driving limits, verify actual motion,
   odometry and single canonical odom/TF ownership using
   [`cleany_base_driver`](../ros2_ws/src/cleany_base_driver/README.md).

Record firmware/Agent revisions, settings, power/load/surface, wheel signs,
command/feedback rates and measured stop times/pass results. Physical robot
acceptance remains required before completing the integration Story.
