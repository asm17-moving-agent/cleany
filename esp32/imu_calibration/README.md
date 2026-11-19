# MPU6050 gyro calibration diagnostic

Standalone ESP32-S3 firmware estimates three sensor-frame gyro biases and
checks them against a separate stationary validation window. Results are
JSON lines over native USB. The diagnostic holds motor PWM/DIR outputs LOW
and runs without motor-control tasks or micro-ROS.

Normal power-on calibration now runs inside the
[motor firmware](../README.md#power-on-calibration), using its build-time
robot config. This standalone application is for acquiring bench reports.

Acquisition/calibration runs in a task pinned to **core 1**; USB command parsing,
JSON serialization, and output run on **core 0**. The IMU task exclusively owns
the driver and calibrator. A length-one overwrite queue transfers immutable
result snapshots; bounded restart/acknowledgement queues carry commands.
USB backpressure never blocks the IMU task.

The default build collects **candidate measurements**, not an accepted
calibration: motion and thermal limits have not yet been measured/reviewed.
It reports `accepted=false`, `error="limits_not_reviewed"`, and
`bias_rad_s=null`. Candidate bias, covariance, temperature, and validation
residuals remain available for selecting those limits.

## Hardware and preparation

- ESP32-S3 N32R16V, matching the motor firmware's octal memory configuration.
- MPU6050 on `J_I2C1`: pin 1 +3V3, pin 2 GND, pin 3 SCL/GPIO42,
  pin 4 SDA/GPIO41. Confirm the pin-1 mark using [PINMAP.md](../PINMAP.md).
- Address `0x68`, `WHO_AM_I=0x68`; module/external 3.3 V I2C pull-ups required.
- Disconnect motor-driver power during flashing and diagnostic runs. MCU/IMU
  power can remain supplied over USB. Output LOW is established at `app_main`;
  it is not a guarantee about hardware pin levels during boot/reset.
- Rigidly mount the sensor and keep the entire robot physically stationary
  throughout settling, calibration, and validation.
- Stop the micro-ROS Agent, serial monitors, and other users of this USB port.

Gyro/acceleration variance checks supplement operator-confirmed rest. Constant
slow rotation can look like gyro bias even with low sample variance.

## Acquisition and calculation

The driver resets the MPU6050, checks identity, enables all axes and temperature,
selects the X-gyro PLL reference, and verifies configured ranges/divider.
It polls data-ready and reads one 14-byte acceleration/temperature/gyro burst
per fresh sample. A missing-ready deadline or I2C error ends the attempt;
old samples are never substituted.

| Setting | Value |
|---|---|
| I2C | 100 kHz, GPIO41 SDA / GPIO42 SCL |
| Sensor sample rate | 100 Hz |
| Accelerometer / gyro ranges | ±2g / ±250 degrees/s |
| Gyro DLPF | 42 Hz (`DLPF_CFG=3`) |
| Default settling interval | 5 seconds |
| Default calibration | 1,000 fresh samples, approximately 10 seconds |
| Default independent validation | 500 new samples, approximately 5 seconds |
| Default I2C transaction timeout | 10 ms |
| Default maximum fresh-sample gap | 30 ms |

Five seconds is an experimental settling interval, not a claim of thermal
equilibrium. Characterize temperature and bias on the actual module and
increase this interval when needed.

Acceleration is decoded in m/s², including gravity; gyro in rad/s.
Accelerometer offsets/scales are not estimated by this gyro calibration.
Each window maintains vector means and full row-major 3×3 sample covariance
using Welford accumulation, without storing the sample history:

```text
candidate_bias = calibration gyro mean
corrected gyro = measured gyro − candidate_bias
validation_residual = independent validation gyro mean − candidate_bias
```

Timestamps are MCU monotonic acquisition microseconds, not ROS epoch.
Nonfinite/saturated samples, non-increasing timestamps, excessive gaps, and
completed windows shorter than 90% of nominal elapsed time fail the attempt.
The covariance is sample measurement covariance, not covariance of the
estimated mean; filtered samples are temporally correlated.

## Build and tests

Use the existing PlatformIO installation prepared by `make firmware-setup`.
From the repository root:

```bash
make test-imu-calibration
make imu-calibration-build
```

Make uses the Ubuntu 22.04 `ros2-humble` Distrobox from the host. This standalone
project does not generate or link micro-ROS type support.
Host tests cover decoding, calibration statistics and failure modes, fake
serial capture, strict accepted-result validation, and configuration refresh.

## Configure measured acceptance limits

Authoritative overrides belong in this project's `sdkconfig.defaults`.
Changing it or `main/Kconfig.projbuild` regenerates derived SDK configuration
on the next build; do not edit generated `sdkconfig.*` files.

| Kconfig setting | Default / units |
|---|---|
| `CLEANY_IMU_SETTLING_MS` | 5000 ms |
| `CLEANY_IMU_CALIBRATION_SECONDS` | 10 s |
| `CLEANY_IMU_VALIDATION_SECONDS` | 5 s |
| `CLEANY_IMU_I2C_TIMEOUT_MS` | 10 ms |
| `CLEANY_IMU_MAX_SAMPLE_GAP_MS` | 30 ms |
| `CLEANY_IMU_LIMITS_REVIEWED` | `n` |
| `CLEANY_IMU_MAX_GYRO_STD_MICRORAD_S` | 0; microrad/s |
| `CLEANY_IMU_MAX_ACCEL_STD_MM_S2` | 0; mm/s² |
| `CLEANY_IMU_MAX_VALIDATION_MEAN_MICRORAD_S` | 0; microrad/s |
| `CLEANY_IMU_MAX_TEMPERATURE_SPAN_MILLIC` | 0; millidegrees C |

First capture repeated stationary candidate reports at the expected operating
temperatures. Review per-axis standard deviations (`sqrt(covariance diagonal)`),
holdout residuals, temperature span, and repeatability. Select and record
positive acceptance limits from those measurements; then set
`CONFIG_CLEANY_IMU_LIMITS_REVIEWED=y` and the four limits and rebuild.
No measured limit values are supplied yet.

With reviewed limits, both windows must pass gyro/acceleration noise limits,
their combined temperature span must pass, and every absolute corrected
validation mean must pass. Missing/nonpositive reviewed limits cause
`invalid_config`. Failed runs never expose an accepted bias.

## Upload and capture

Upload replaces the motor-controller application. Select the exact device and
explicitly confirm upload after completing hardware preparation:

```bash
export CLEANY_ESP_PORT=/dev/serial/by-id/<native-USB-device>
make imu-calibration-upload CLEANY_ESP_PORT="$CLEANY_ESP_PORT" CONFIRM_UPLOAD=1
make imu-calibration-capture CLEANY_ESP_PORT="$CLEANY_ESP_PORT" \
  IMU_CALIBRATION_OUTPUT=artifacts/imu_calibration/run1.json
```

Calibration starts once at boot. `status` returns progress or the retained
terminal result, so attaching after completion works. To restart a stationary
attempt explicitly:

```bash
make imu-calibration-capture CLEANY_ESP_PORT="$CLEANY_ESP_PORT" \
  IMU_CALIBRATION_ARGS=--calibrate \
  IMU_CALIBRATION_OUTPUT=artifacts/imu_calibration/run2.json
```

Capture uses exclusive POSIX serial access and atomically deasserts DTR/RTS
during opening. Sequential pySerial control-line updates caused MCU resets on
the bench ESP32-S3; atomic updates preserved its run ID and monotonic timebase.
`--calibrate` sends one command and
waits for that command's run acknowledgement. Periodic `status` queries recover
a retained result if its automatic transmission was missed.
Each host restart carries a random request nonce; old buffered command
acknowledgements/results cannot satisfy a new capture.
Read/write operations and the overall capture are bounded; the default
overall timeout is 60 seconds, configurable with `IMU_CALIBRATION_TIMEOUT`.
Increase it for longer configured settling/windows.

Exit codes: **0** accepted report, **2** captured rejected/unreviewed report,
**1** capture/serial/protocol error. Exit 2 still saves the report; Make returns
nonzero for that measurement-only default. Output is atomically written.

## USB protocol and results

Commands are newline-terminated `status`, `calibrate`, or
`calibrate <16-lowercase-hex-digit-request-id>`. Capture uses the nonce form.
Messages use
`schema="cleany.imu_calibration/v1"`:

- `started`: `trigger` (`boot` or `command`), new `run_id`, and `request_id`
  (empty for boot or a manual nonce-free command).
- `status`: phase and current calibration/validation sample counts.
- `result`: retained terminal report.
- `error`: malformed/unknown command.

Terminal reports include `phase` (`completed` or `failed`), `accepted`, `error`,
sensor identity/error/operation, configuration, both windows' means/covariances/
timestamps/temperature ranges, `candidate_bias_rad_s`, `bias_rad_s`, and
`validation_residual_rad_s`. Vectors are sensor-frame X/Y/Z; covariance arrays
are flat row-major 3×3. `completed` means acquisition finished; only
`accepted=true` indicates validated calibration. `bias_rad_s` is otherwise null.

Results remain in RAM until a new attempt or reset; no NVS calibration is
written. Saved JSON is an inspection record, not automatically applied to
future boots or firmware. A new power cycle requires a new bias estimate.

## Hardware acceptance and restoration

After upload, inspect stationary candidates and repeat runs. With reviewed
limits, test deliberate motion, sensor disconnection, and a temperature-changing
run; verify rejection, then a fresh stationary retry. Check PWM/DIR levels
physically with motor-driver power disconnected. These hardware acceptance
checks are separate from host tests and successful firmware compilation.

Close capture before restoring the motor firmware:

```bash
make firmware-upload CLEANY_ESP_PORT="$CLEANY_ESP_PORT" CONFIRM_UPLOAD=1
```

Apply the normal [motor firmware acceptance procedure](../README.md#robot-acceptance)
before restoring motor-driver power and enabling motion.

## Bench measurement record

On 2026-10-14, six diagnostic acquisitions on ESP32-S3
`90:e5:b1:d5:3f:34` completed with 1,000 calibration and 500 validation samples
each, `WHO_AM_I=0x68`, and no reported I2C/timing failure. Local JSON reports,
upload evidence, and detailed findings are under
`artifacts/imu_calibration/bench_20261014T142933Z/`.

The latest candidate bias was `[0.00145129, 0.02294135, -0.02154442]` rad/s
(sensor X/Y/Z); its independent corrected validation mean was
`[0.00001079, 0.00000826, 0.00002465]` rad/s at 27.824–27.901 °C.
These remain unreviewed candidate measurements, not persisted calibration.
Across the session, bias varied and mean acceleration magnitude was
approximately 0.904–0.907g; verify accelerometer offsets/scales separately.

Atomic host control-line updates preserved the MCU timebase across two
consecutive acquisitions and two subsequent serial reconnects.
