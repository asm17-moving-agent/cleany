# 통합 컨트롤러 핀아웃 및 인터페이스 규격

본 문서는 통합 컨트롤러(`2.9-draft`)의 입출력 신호 할당, 커넥터 결선, 전원 계통 및 인터페이스 전기적 규격을 정의한다. 상세 부품 명세는 [BOM.md](BOM.md) 및 [BOM.csv](BOM.csv)를 참조한다.

## 1. 시스템 개요

- **도면 체계**: 루트 회로도([integrated_controller.kicad_sch](integrated_controller.kicad_sch)), 전원([power.kicad_sch](power.kicad_sch)), 센서([sensors.kicad_sch](sensors.kicad_sch)), 외부 인터페이스([interfaces.kicad_sch](interfaces.kicad_sch))
- **메인 MCU**: ESP32-S3-WROOM-2-N32R16V (32 MB Octal-SPI Flash, 16 MB Octal-SPI PSRAM)
- **센서 인터페이스**: 온보드 BNO055 IMU와 외부 ToF 센서 4개용 I2C 멀티플렉서.
- **로직 레벨**: 3.3 V LVCMOS (Native USB D+/D− 제외)
- **모터 좌표계**: 로봇 전방 기준 M1(전방 좌측), M2(전방 우측), M3(후방 우측), M4(후방 좌측)

## 2. MCU 핀 할당 (ESP32-S3)

방향 기준은 ESP32-S3 MCU 입출력이다.

| 신호명 | GPIO | 패드 | 입출력 | 연결 대상 및 설명 |
|---|---:|---:|---|---|
| M1_PWM | 8 | 12 | Output | 모터 1 속도 제어 PWM |
| M1_DIR | 18 | 11 | Output | 모터 1 회전 방향 제어 |
| M1_ENC_A | 17 | 10 | Input | 모터 1 엔코더 Phase A |
| M1_ENC_B | 16 | 9 | Input | 모터 1 엔코더 Phase B |
| M2_PWM | 15 | 8 | Output | 모터 2 속도 제어 PWM |
| M2_DIR | 7 | 7 | Output | 모터 2 회전 방향 제어 |
| M2_ENC_A | 6 | 6 | Input | 모터 2 엔코더 Phase A |
| M2_ENC_B | 5 | 5 | Input | 모터 2 엔코더 Phase B |
| M3_PWM | 12 | 20 | Output | 모터 3 속도 제어 PWM |
| M3_DIR | 11 | 19 | Output | 모터 3 회전 방향 제어 |
| M3_ENC_A | 10 | 18 | Input | 모터 3 엔코더 Phase A |
| M3_ENC_B | 9 | 17 | Input | 모터 3 엔코더 Phase B |
| M4_PWM | 38 | 31 | Output | 모터 4 속도 제어 PWM |
| M4_DIR | 21 | 23 | Output | 모터 4 회전 방향 제어 |
| M4_ENC_A | 14 | 22 | Input | 모터 4 엔코더 Phase A |
| M4_ENC_B | 13 | 21 | Input | 모터 4 엔코더 Phase B |
| I2C0_SDA | 1 | 39 | In/Out | TCA9548A Upstream SDA (오픈 드레인) |
| I2C0_SCL | 2 | 38 | Output | TCA9548A Upstream SCL (오픈 드레인) |
| I2C1_SDA | 41 | 34 | In/Out | BNO055 SDA (오픈 드레인) |
| I2C1_SCL | 42 | 35 | Output | BNO055 SCL (오픈 드레인) |
| LED_DATA | 4 | 4 | Output | 주소지정형 LED 데이터 출력 (3.3 V 레벨) |
| USB_D_N | 19 | 13 | In/Out | Native USB Serial/JTAG D− |
| USB_D_P | 20 | 14 | In/Out | Native USB Serial/JTAG D+ |
| BOOT_N | 0 | 27 | Input | 부트스트랩 / 펌웨어 다운로드 스위치 (SW1) |
| BAT_ADC | 3 | 15 | Input | 4S LiFePO4 배터리 전압, ADC1_CH2 |

> [!NOTE]
> - BNO055와 TCA9548A의 RESET 핀은 각각 R13과 R19(각 10 kΩ)로 +3V3에 풀업되며 MCU와 연결하지 않는다.
> - BNO055의 인터럽트 핀(INT)은 NC(No Connect)이며 데이터는 폴링 방식으로 취득한다.
> - ToF 센서 커넥터에는 개별 XSHUT 및 하드웨어 리셋 라인을 배치하지 않는다.

M1/M2는 모듈 왼쪽의 패드 5–12에, M3는 하단 패드 17–20에 모은다. M4는 하단 패드 21–23과 오른쪽 패드 31을 사용한다. LED_DATA는 왼쪽 패드 4, IMU 버스는 오른쪽 패드 34–35, ToF 버스는 오른쪽 패드 38–39에 배치한다. USB GPIO19/20, BOOT GPIO0 및 외부 커넥터 결선은 고정한다.

### MCU 전원 및 제어 패드

| 패드 번호 | 신호명 | 기능 |
|---|---|---|
| 2 | +3V3 | MCU 로직 전원 입력 (3.0–3.6 V) |
| 1, 40, 41 | GND | 공통 접지 (41: 바닥 노출 방열 패드) |
| 3 | CHIP_PU | 칩 인에이블 / 하드웨어 리셋 (SW2 및 RC 지연 회로 연결) |

## 3. 외부 커넥터 규격

### 3.1 커넥터 목록

| 식별 부호 | 기능 | 부품 규격 | 결합 형식 |
|---|---|---|---|
| SW3 | 3.3 V 로직 인에이블 접점 | Molex `0022057025` (`5268-02A`) | Mini-SPOX 2P 수평 직각 헤더 (2.50 mm 피치) |
| J1 | 배터리 주 전원 입력 | 수직형 XT60 수형 (2P) | 패드 간격 7.20 mm, 홀 직경 Ø4.50 mm |
| J2 | Native USB 인터페이스 | On Shore `USB-B1HSB6` | USB Type-B 수평 직각, 신호핀 4개와 실드 고정핀 2개 모두 THT |
| J3 | LED 1 전원 및 데이터 | Phoenix Contact `1989450` | 3P 스크루 단자대 (5.00 mm 피치) |
| J4–J6 | LED 2–4 전원 출력 | Phoenix Contact `1935161` | 2P 스크루 단자대 (5.00 mm 피치) |
| J7–J10 | 모터 제어 신호 (M1–M4) | Molex `0022057035` (`5268-03A`) | Mini-SPOX 3P 수평 직각 헤더 (2.50 mm 피치) |
| J11–J14 | 모터 엔코더 입력 (M1–M4) | Molex `0022057045` (`5268-04A`) | Mini-SPOX 4P 수평 직각 헤더 (2.50 mm 피치) |
| J15–J18 | ToF 센서 버스 (ToF 1–4) | Assmann `A-2004-3-4-LP-N-R` | 4P4C (RJ9/RJ22) 수평 모듈러 잭 |

### 3.2 전원 입력 및 Native USB 회로 (J1, J2)

배터리 전원(RAW)과 USB VBUS는 쇼트키 다이오드 OR로 VIN_LOGIC에 합쳐져 3.3 V 로직을 공급한다. 5 V LED 레일(U4–U7)은 RAW에서만 공급된다.

```text
배터리 주 입력 (J1) ──> D1 (SS34) ──────────────┐
J2 VBUS ──> JP1 (USB_PWR 점퍼) ──> D2 (SS34) ──┴──> VIN_LOGIC ──> U3 (3.3V Buck) ──> +3V3
Jetson USB Host ──> J2 ──> U2 (USBLC6-2SC6) ──> ESP32-S3 (GPIO19/20)
J2 VBUS ─────────> U2 Pin 5 (ESD 보호 레일 바이어스, C3 100 nF 바이패스)
```

| 인터페이스 | 설계 사양 |
|---|---|
| USB 데이터 라인 | GPIO19(D−) 및 GPIO20(D+) 직결. 과도 전압 억제용 U2(`USBLC6-2SC6`) 배치 |
| USB-B 핀 배치 | 1: VBUS, 2: D−, 3: D+, 4: GND, SH: GND |
| VBUS 처리 | U2 Pin 5 바이어스에 연결하고, JP1과 D2를 거쳐 VIN_LOGIC에 공급. D2가 VIN_LOGIC에서 VBUS로의 역급전을 차단 |
| JP1 (USB_PWR) | 쇼트 점퍼 장착 시 USB 급전 경로 연결. 점퍼를 제거하면 USB는 데이터와 ESD 바이어스 용도로만 남음 |

USB 차동 쌍은 J2–U2 구간의 `/USB_CONN_N`, `/USB_CONN_P`와 U2–MCU 구간의 `USB_D_N`, `USB_D_P`다. 각 쌍은 KiCad 차동 배선 인식을 위해 같은 접두사에 `_N`, `_P` 접미사를 사용한다.

> [!CAUTION]
> JP1을 장착하고 SW3의 외부 접점을 닫은 상태에서 USB 단독 연결 시 3.3 V 로직(MCU, 센서, 엔코더 전원)만 기동되고 5 V LED 레일은 꺼져 있다. 배터리와 USB가 함께 연결되면 전압이 높은 RAW가 VIN_LOGIC을 공급한다. 호스트와 보드는 USB 케이블 실드 및 GND 핀을 통해 접지를 공유한다.

SW3는 외부 접점 스위치를 연결하는 헤더다. 접점을 닫으면 U3의 EN이 VIN_LOGIC에 연결되고, 열면 R4(100 kΩ)가 EN을 GND로 당긴다. 3.3 V 로직만 제어하며 RAW와 5 V LED 레일은 차단하지 않는다.

| SW3 핀 | 연결 | 의미 |
|---|---|---|
| 1 | VIN_LOGIC | 입력 전압. +3V3 출력이 아님 |
| 2 | U3 EN (핀 3), R4 핀 2 | 외부 접점으로 핀 1과 연결 |

R4 핀 1은 GND다. 외부 접점은 VIN_LOGIC 전압에 맞는 정격을 사용한다.

### 3.3 모터 제어 및 엔코더 인터페이스 (J7–J14)

모터 커넥터는 신호 전용 인터페이스이며 모터 구동 전력을 인가하지 않는다.

| 커넥터 | 핀 1 | 핀 2 | 핀 3 | 핀 4 |
|---|---|---|---|---|
| J7 (MOTOR1) | M1_PWM | GND | M1_DIR | - |
| J8 (MOTOR2) | M2_PWM | GND | M2_DIR | - |
| J9 (MOTOR3) | M3_PWM | GND | M3_DIR | - |
| J10 (MOTOR4) | M4_PWM | GND | M4_DIR | - |
| J11 (ENCODER1) | +3V3 | GND | M1_ENC_A | M1_ENC_B |
| J12 (ENCODER2) | +3V3 | GND | M2_ENC_A | M2_ENC_B |
| J13 (ENCODER3) | +3V3 | GND | M3_ENC_A | M3_ENC_B |
| J14 (ENCODER4) | +3V3 | GND | M4_ENC_A | M4_ENC_B |

- 결합 하우징: Molex `5264-03`(신호용 3P), `5264-04`(엔코더용 4P)
- 엔코더 인터페이스 전압: +3.3 V LVCMOS 입력.

### 3.4 ToF 센서 인터페이스 (J15–J18, 4P4C)

TCA9548A 멀티플렉서의 개별 채널을 4P4C 모듈러 커넥터로 인출한다.

U9의 RESET(핀 3)은 R19의 풀업으로 HIGH를 유지한다. MCU 리셋 스위치 SW2는 U9를 리셋하지 않는다. 버스 고착 등으로 U9의 하드웨어 리셋이 필요하면 +3V3 전원을 껐다 켜야 한다.

| 커넥터 | 채널 | 핀 1 | 핀 2 | 핀 3 | 핀 4 |
|---|---:|---|---|---|---|
| J15 (ToF 1) | CH0 | GND | TOF1_SCL | TOF1_SDA | +3V3 |
| J16 (ToF 2) | CH1 | GND | TOF2_SCL | TOF2_SDA | +3V3 |
| J17 (ToF 3) | CH2 | GND | TOF3_SCL | TOF3_SDA | +3V3 |
| J18 (ToF 4) | CH3 | GND | TOF4_SCL | TOF4_SDA | +3V3 |

- **버스 풀업 사양**:
  - TCA9548A Upstream 버스(I2C0): 4.7 kΩ 풀업 (R17, R18)
  - 센서 보드 Downstream: 채널별 센서 모듈 내 2.2 kΩ 풀업 실장
  - 개별 채널 인에이블 시 등가 풀업 저항: 약 1.5 kΩ 병렬 형성
- **하네스 결선 규격**: 4P4C 플러그 양단 1:1 직결 결선 (Pin 1 ↔ Pin 1, Pin 2 ↔ Pin 2, Pin 3 ↔ Pin 3, Pin 4 ↔ Pin 4)

### 3.5 주소지정형 LED 인터페이스 (J3–J6)

4계통 독립 전원 및 단일 직렬 데이터 체인 구성이다.

| 커넥터 | 핀 1 | 핀 2 | 핀 3 | 공급 전원 레일 |
|---|---|---|---|---|
| J3 (LED 1) | +5V_LED1 | GND | LED_DIN | U4 (5 V / 4 A Buck) |
| J4 (LED 2) | +5V_LED2 | GND | - | U5 (5 V / 4 A Buck) |
| J5 (LED 3) | +5V_LED3 | GND | - | U6 (5 V / 4 A Buck) |
| J6 (LED 4) | +5V_LED4 | GND | - | U7 (5 V / 4 A Buck) |

- **데이터 신호 변환 회로 (U10: SN74AHCT1G125DCKR)**:
  - 3.3 V → 5.0 V 논리 레벨 시프터 (단방향 버퍼)
  - 전원 입력: +5V_LED1 (동작 전압 범위: 4.5–5.5 V)
  - 제어 신호: `/OE` (Pin 1)는 GND에 고정 결선되어 상시 활성화
  - 입력단 안정화: R21(10 kΩ 풀다운 저항) 실장
  - 출력단 댐핑: R20(330 Ω 직렬 저항)을 통해 J3 Pin 3(LED_DIN)으로 출력

> [!WARNING]
> 4개 LED 전원 레일(+5V_LED1–4)은 각각 독립된 벅 컨버터 출력이므로 외부에서 양극 라인을 병렬 연결하지 않는다. GND 라인은 단일 공통 접지를 형성한다.

## 4. 전원 분배 및 센서 회로 제원

### 4.1 DC-DC 벅 레귤레이터 (LMR33640DDDAR)

5계통의 스위칭 레귤레이터 회로는 동일한 1 MHz 표준 토폴로지를 적용한다.

- 스위칭 주파수: 1.0 MHz
- 인덕터(L1–L5): 3.3 µH (Würth `74439346033`, Ir=8 A, Isat=7.5 A)
- 입력 커패시터: 10 µF 50 V X5R 1206 (C5, C12, C19, C26, C33) + 220 nF 50 V X7R 0603 (C6, C13, C20, C27, C34)
- 출력 커패시터: 22 µF 16 V X7R 1210 × 3병렬 (총 66 µF 공칭 용량)
- 부트스트랩 커패시터: 100 nF 25 V X7R 0603 (C8, C15, C22, C29, C36)
- 내부 LDO 바이패스: 1 µF 16 V X7R 0603 (C7, C14, C21, C28, C35)

| 레귤레이터 | 입력 전원 | 출력 레일 | 목표 전압 | FB 상단 저항 | FB 하단 저항 | 용도 |
|---|---|---|---:|---|---|---|
| U3 | VIN_LOGIC | +3V3 | 3.30 V | 100 kΩ ±1% | 43.2 kΩ ±1% | 로직, MCU, 센서 및 엔코더 전원 |
| U4 | RAW | +5V_LED1 | 5.02 V | 100 kΩ ±1% | 24.9 kΩ ±1% | LED 1 전원 및 U10 버퍼 전원 |
| U5 | RAW | +5V_LED2 | 5.02 V | 100 kΩ ±1% | 24.9 kΩ ±1% | LED 2 전원 |
| U6 | RAW | +5V_LED3 | 5.02 V | 100 kΩ ±1% | 24.9 kΩ ±1% | LED 3 전원 |
| U7 | RAW | +5V_LED4 | 5.02 V | 100 kΩ ±1% | 24.9 kΩ ±1% | LED 4 전원 |

### 4.2 센서 및 통신 IC 구성

| 식별자 | 부품 모델 | I2C 기본 주소 | 패키지 규격 | 상세 사양 |
|---|---|---|---|---|
| U8 | BNO055 | 0x28 | 28-LGA (5.2 × 3.8 mm) | 9축 IMU, 내부 발진기 구동, 3.3 V 전원 레일 |
| U9 | TCA9548ADGSR | 0x70 | 24-VSSOP (DGS, 0.5 mm 피치) | 8채널 I2C 스위치, CH0–CH3 사용 (CH4–CH7 미사용) |

U8의 RESET(핀 11)은 R13의 풀업으로 HIGH를 유지한다. SW2는 ESP32만 리셋하며 U8과 U9의 RESET 핀을 구동하지 않는다.

U8 주변 배치는 [BNO055 Xplained Pro 공식 설계 자료](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/BoardDesignFiles/BNO055-Xplained-Pro_Design-Documentation.zip)의 Rev3 CAD와 `Pick Place for BNO055_Xplained_pro.txt`를 기준으로 한다. 평가보드 U101의 축을 U8의 KiCad 회전 0°에 맞춰 상대 좌표를 변환했다. 부품 대응은 C40↔C105(VDD), C41↔C103(VDDIO), C42↔C108(CAP), R13↔R112(RESET), R14↔R106(BOOT), R15↔R101(SDA), R16↔R102(SCL)이다. 값, 풋프린트와 실장 여부는 본 프로젝트 회로도를 따른다.

평가보드의 0402 부품을 현재 0603 풋프린트로 유지하면서 생기는 간섭은 간격 확대로 해소했다. 변환한 공식 중심 좌표에서 C40은 X −0.35 mm, R14는 Y +0.40 mm, R13은 X +0.55 mm와 Y +0.30 mm, R16은 Y +0.45 mm 이동했다. C41, C42와 R15의 상대 중심 좌표는 유지했고, 저항의 패드 번호 차이는 해당 신호와 전원 연결 방향으로 맞췄다. 나머지 평가보드 부품은 추가하지 않는다.

### 4.3 배터리 전압 측정

4S LiFePO4 배터리의 RAW 전압을 J1 및 D1 앞에서 분기한다. R22(100 kΩ)는 RAW와 BAT_ADC 사이, R23(22 kΩ)와 C45(100 nF)는 BAT_ADC와 GND 사이에 연결한다. BAT_ADC는 GPIO3(U1 패드 15, ADC1_CH2)에 연결한다. 저항은 각각 ±1%, 1608이며 C45는 구매한 KEMET C0603C104K5RACTU 여분을 사용한다.

| 항목 | 값 |
|---|---|
| 전압 환산 | Vbat = 보정된 Vadc × 122 / 22 |
| 14.6 V 입력 | 공칭 2.633 V, 저항 오차만 고려한 최대 2.676 V |
| 분압기 소비 전류 | 14.6 V에서 약 120 µA |
| 필터 시정수 | (100 kΩ ∥ 22 kΩ) × 100 nF ≈ 1.80 ms |

펌웨어는 `ADC_ATTEN_DB_12`와 ADC 보정을 적용하고 여러 샘플을 평균한다. 기준 전압계로 분압비와 ADC 오차를 함께 교정한다. ESP32-S3의 [ADC 보정 및 잡음 감소 지침](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc/index.html)을 따른다. 배터리 전압만으로 LiFePO4 잔량 백분율을 확정하지 않는다.

SW3로 +3V3를 꺼도 분압기는 RAW에 연결되어 있다. 별도 역급전 차단은 없으므로 GPIO3을 통한 OFF 상태 전류 유입과 전원 상승·하강 동작을 실물에서 확인한다. 100 kΩ는 이 경로의 전류를 제한하지만 전원 OFF 핀 허용 정격을 보장하지 않는다. 이 회로는 배터리 보호나 충전 제어를 수행하지 않는다.

GPIO3는 JTAG 선택 스트랩과 겸용이다. [모듈 데이터시트의 JTAG 선택 표](https://documentation.espressif.com/esp32-s3-wroom-2_datasheet_en.html)에 따라 기본 eFuse 설정에서는 GPIO3 스트랩을 무시한다. JTAG 관련 eFuse 설정을 바꿀 때 이 아날로그 입력을 고정 논리값으로 가정하지 않는다.

## 5. 핀 제한 사항 및 예약 핀

| GPIO | 상태 | 설계 제약 사유 |
|---|---|---|
| 0 | 시스템 예약 | BOOT 기능 전용 (기동 시 스트랩 핀) |
| 3 | BAT_ADC | JTAG 선택 스트랩 겸용. 기본 eFuse에서는 스트랩 값을 무시하며 USB Serial/JTAG 사용 |
| 45, 46 | 미할당 | MCU 기동 및 VDD_SPI 전압 결정용 스트랩 핀 |
| 35, 36, 37 | 사용 불가 | Octal 메모리용 내부 인터페이스이며 WROOM-2 모듈 패드 28–30은 NC |
| 43, 44 | 미할당 (NC) | 기본 ROM 콘솔 UART0 (TXD0, RXD0) 예비 라인 |
| 47, 48 | 미할당 (NC) | N32R16V에서 1.8 V GPIO이며 3.3 V 주변장치에 직접 연결 금지 |
| 39, 40 | 미할당 (NC) | 모듈 패드 32–33. 주변장치 연결 없이 남겨 둠 |
| 41, 42 | 주변장치 할당 | 모듈 패드 34–35. BNO055 I2C 통신 전용 |

## 6. 펌웨어 인터페이스 요건

- **I2C 컨트롤러 할당**:
  - `I2C_NUM_0`: TCA9548A 멀티플렉서 제어 (SDA: GPIO1, SCL: GPIO2)
  - `I2C_NUM_1`: BNO055 IMU 통신 전용 (SDA: GPIO41, SCL: GPIO42)
- **통신 파라미터**:
  - 기본 버스 주파수: 100 kHz 표준 모드
  - BNO055 기동 시 클럭 스트레칭(Clock Stretching) 처리 루틴 필수
- **GPIO 출력 초기화**:
  - 기동 시 GPIO4(LED_DATA)은 LOW 상태로 유지
  - 레귤레이터 안정화 완료 후 데이터 프레임 전송 개시

기존 `esp32/src/main.cpp`의 모터 GPIO 배열은 별도 `motor_controller` 보드를 대상으로 한다. 통합 컨트롤러용 펌웨어는 위 표를 적용해야 하며, 기존 이미지의 핀 할당과 호환되지 않는다. 통합 보드에는 별도 펌웨어 빌드 프로파일이 아직 없다.

## 7. 참조 규격

- Espressif Systems: [ESP32-S3-WROOM-2 Datasheet v1.7](https://documentation.espressif.com/esp32-s3-wroom-2_datasheet_en.pdf), Table 3-1 및 §4의 핀 전압과 스트랩 조건
- Texas Instruments: LMR33640 Datasheet (SNVSB99C) / LMR33640EVM User's Guide (SNVU664)
- Texas Instruments: SN74AHCT1G125 Datasheet (SCLS377P)
- Texas Instruments: [TCA9548A Datasheet](https://www.ti.com/lit/ds/symlink/tca9548a.pdf) (SCPS207H), Table 4-1 및 §7.4의 RESET 풀업과 전원 리셋 조건
- Bosch Sensortec: BNO055 Datasheet (BST-BNO055-DS000-18)
- STMicroelectronics: USBLC6-2SC6 Datasheet (DocID9535 Rev 10)
- Comchip Technology: SS34-HF Datasheet
