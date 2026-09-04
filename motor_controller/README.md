# ESP32-S3 motor and IMU hardware tests

The normal PlatformIO firmware entry point is `src/main.cpp`. Hardware checks
are independent Unity applications under `test/`, so a test runs only when it
is explicitly selected with `pio test`.

The PlatformIO environment targets the ESP32-S3-DevKitC-1-N32R16V with 32 MB
octal flash and 16 MB octal PSRAM.

Build and upload the non-test firmware with PlatformIO isolated by `uv`:

```bash
uvx --with pip --from platformio pio run -d motor_controller
uvx --with pip --from platformio pio run -d motor_controller --target upload \
  --upload-port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:E5:B1:D5:3F:34-if00
uvx --with pip --from platformio pio device monitor -d motor_controller \
  --port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:E5:B1:D5:3F:34-if00 \
  --baud 115200
```

## Hardware tests

Run both suites in sequence with PlatformIO port auto-detection:

```bash
uvx --with pip --from platformio pio test -d motor_controller -e esp32s3
```

If the board was left disconnected by an older test build, perform the
BOOT/RESET upload sequence once before running this command. Test builds from
this project keep USB CDC active between suites, so subsequent uploads should
not require that sequence.

Set the connected board port once:

```bash
CLEANY_ESP_PORT=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_90:E5:B1:D5:3F:34-if00
```

Run only the motor test:

```bash
uvx --with pip --from platformio pio test -d motor_controller -e esp32s3 \
  -f test_motor --upload-port "$CLEANY_ESP_PORT" --test-port "$CLEANY_ESP_PORT"
```

The motor test uses encoder A on GPIO4, encoder B on GPIO3, direction on
GPIO10, and PWM on GPIO11. It applies 50% duty for at most three seconds and
always stops before reporting whether 793 encoder counts were reached. The
motor supply, driver, encoder, and development board must share ground.

Run only the MPU6050 test:

```bash
uvx --with pip --from platformio pio test -d motor_controller -e esp32s3 \
  -f test_imu --upload-port "$CLEANY_ESP_PORT" --test-port "$CLEANY_ESP_PORT"
```

Connect MPU6050 VCC to 3V3, GND to GND, SDA to GPIO8, and SCL to GPIO9. The
test accepts address `0x68` or `0x69`, checks `WHO_AM_I`, wakes the device,
and reads a complete accelerometer, temperature, and gyroscope frame.

The project-local `test/unity_config.h` keeps the ESP32-S3 native USB CDC
interface active after `UNITY_END()`. This allows PlatformIO to upload the
next test suite without another manual BOOT/RESET sequence.

## KiCad symbol and footprint

The project-local KiCad library tables register Espressif's official
ESP32-S3-DevKitC symbol and footprint as
`PCM_Espressif:ESP32-S3-DevKitC`. The vendored source revision, mechanical
drawing used for verification, and license are recorded in
[`libraries/README.md`](libraries/README.md).
