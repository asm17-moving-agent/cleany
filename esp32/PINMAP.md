# Motor controller pin map

This pin map is derived from the electrical connections in
[`pcb/motor_controller.kicad_sch`](pcb/motor_controller.kicad_sch) and cross-checked
against the pad nets in
[`pcb/motor_controller.kicad_pcb`](pcb/motor_controller.kicad_pcb). The schematic and
PCB assignments match. Connector pin numbers are KiCad symbol/pad numbers. On
the PCB, pin 1 has a round-rectangle pad; the remaining connector pads are
oval. Verify the pin-1 mark before making a cable, since the order appears
reversed when a connector is viewed from its mating side.

All connector logic and encoder power rails are 3.3 V. The `J_MOTOR*`
connectors carry PWM and direction control signals, not motor phase current.

## ESP32-S3 signal assignment

Motor placement, viewed from above with the robot front facing forward, is
M1 front-left, M2 front-right, M3 rear-right, and M4 rear-left.

| Function | ESP32-S3 GPIO | U1 pad | Connector pin |
|---|---:|---:|---|
| Motor 1 PWM | GPIO2 | 40 | `J_MOTOR1.1` |
| Motor 1 direction | GPIO1 | 41 | `J_MOTOR1.3` |
| Encoder 1 A | GPIO10 | 16 | `J_ENC1.3` |
| Encoder 1 B | GPIO9 | 15 | `J_ENC1.4` |
| Motor 2 PWM | GPIO12 | 18 | `J_MOTOR2.1` |
| Motor 2 direction | GPIO11 | 17 | `J_MOTOR2.3` |
| Encoder 2 A | GPIO14 | 20 | `J_ENC2.3` |
| Encoder 2 B | GPIO13 | 19 | `J_ENC2.4` |
| Motor 3 PWM | GPIO5 | 5 | `J_MOTOR3.1` |
| Motor 3 direction | GPIO6 | 6 | `J_MOTOR3.3` |
| Encoder 3 A | GPIO7 | 7 | `J_ENC3.3` |
| Encoder 3 B | GPIO15 | 8 | `J_ENC3.4` |
| Motor 4 PWM | GPIO16 | 9 | `J_MOTOR4.1` |
| Motor 4 direction | GPIO17 | 10 | `J_MOTOR4.3` |
| Encoder 4 A | GPIO18 | 11 | `J_ENC4.3` |
| Encoder 4 B | GPIO8 | 12 | `J_ENC4.4` |
| I2C 1 SCL | GPIO42 | 39 | `J_I2C1.3` |
| I2C 1 SDA | GPIO41 | 38 | `J_I2C1.4` |
| I2C 2 SCL | GPIO21 | 27 | `J_I2C2.3` |
| I2C 2 SDA | GPIO4 | 4 | `J_I2C2.4` |
| J1 pin 1 (unnamed net) | GPIO40 | 37 | `J1.1` |
| J1 pin 2 (unnamed net) | GPIO39 | 36 | `J1.2` |

## Connector pinouts

### Motor control

All four motor-control headers use the same order.

| Connector | Pin 1 | Pin 2 | Pin 3 |
|---|---|---|---|
| `J_MOTOR1` | M1 PWM / GPIO2 | GND | M1 direction / GPIO1 |
| `J_MOTOR2` | M2 PWM / GPIO12 | GND | M2 direction / GPIO11 |
| `J_MOTOR3` | M3 PWM / GPIO5 | GND | M3 direction / GPIO6 |
| `J_MOTOR4` | M4 PWM / GPIO16 | GND | M4 direction / GPIO17 |

### Encoders

| Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 |
|---|---|---|---|---|
| `J_ENC1` | +3V3 | GND | Encoder 1 A / GPIO10 | Encoder 1 B / GPIO9 |
| `J_ENC2` | +3V3 | GND | Encoder 2 A / GPIO14 | Encoder 2 B / GPIO13 |
| `J_ENC3` | +3V3 | GND | Encoder 3 A / GPIO7 | Encoder 3 B / GPIO15 |
| `J_ENC4` | +3V3 | GND | Encoder 4 A / GPIO18 | Encoder 4 B / GPIO8 |

### I2C

| Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 |
|---|---|---|---|---|
| `J_I2C1` | +3V3 | GND | SCL / GPIO42 | SDA / GPIO41 |
| `J_I2C2` | +3V3 | GND | SCL / GPIO21 | SDA / GPIO4 |

The PCB contains no discrete I2C pull-up resistors. Use pull-ups provided by
the attached module or add appropriate external pull-ups.

### Other headers

| Connector | Pin 1 | Pin 2 |
|---|---|---|
| `J1` | GPIO40, unnamed net | GPIO39, unnamed net |
| `J2` | +3V3 | GND |

`J1` has no functional net labels in the design, so its intended protocol is
not defined by the PCB.

## U1 power and unused pins

| U1 pad(s) | DevKitC pin | PCB connection |
|---|---|---|
| 1, 2 | 3V3 | +3V3 |
| 21 | 5V | VBUS |
| 22, 23, 24, 44 | GND | GND |
| 3 | CHIP_PU | Not connected |
| 13 | GPIO3 | Not connected |
| 14 | GPIO46 | Not connected |
| 25, 26 | GPIO19 / USB D-, GPIO20 / USB D+ | Not connected |
| 28–35 | GPIO47, GPIO48, GPIO45, GPIO0, GPIO35–GPIO38 | Not connected |
| 42, 43 | GPIO44 / U0RXD, GPIO43 / U0TXD | Not connected |

The current standalone hardware tests described in `README.md` use temporary
bench-wiring GPIO assignments. They do not represent this PCB pin map.
