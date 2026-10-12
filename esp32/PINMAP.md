# Motor controller pin map

[Schematic](pcb/motor_controller.kicad_sch)과 [PCB](pcb/motor_controller.kicad_pcb)의 배선, pad net을 기준으로 정리한 pin map입니다. Connector 번호는 KiCad symbol/pad 기준입니다.

Pin 1은 round-rectangle pad, 나머지는 oval pad입니다. Cable을 제작할 때 pin-1 mark를 확인하고 mating side에서 좌우 순서가 반전되는 점을 고려합니다. Logic, encoder 전원은 3.3 V이며, `J_MOTOR`는 PWM/DIR 신호용으로 모터 phase current를 전달하지 않습니다.

## ESP32-S3 signal assignment

로봇을 위에서 보고 전방을 기준으로 M1은 앞 왼쪽, M2는 앞 오른쪽, M3는 뒤 오른쪽, M4는 뒤 왼쪽입니다.

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

네 motor-control header는 같은 pin 순서를 사용합니다.

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

PCB에는 별도 I2C pull-up 저항이 없습니다. 연결 모듈의 pull-up을 사용하거나 적절한 외부 pull-up을 추가합니다.

### Other headers

| Connector | Pin 1 | Pin 2 |
|---|---|---|
| `J1` | GPIO40, unnamed net | GPIO39, unnamed net |
| `J2` | +3V3 | GND |

`J1`에는 기능을 지정한 net label이 없으므로 PCB만으로 사용 프로토콜을 정할 수 없습니다.

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
