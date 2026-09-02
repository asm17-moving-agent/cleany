# ESP32-S3 motor and IMU hardware tests

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
M2 and M3 use normal polarity. Verify all wheel directions with the robot
lifted before driving it. Each active motor must receive the browser's 250 ms
heartbeat and is stopped by the firmware after 750 ms without a command.
Releasing a drive button stops all four motors. A Wi-Fi client disconnect also
stops all motors. GPIO assignments come from [`PINMAP.md`](PINMAP.md).

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
GPIO10, and PWM on GPIO11. It applies 50% duty for at most three seconds and
always stops before reporting whether 793 encoder counts were reached. The
motor supply, driver, encoder, and development board must share ground.

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
