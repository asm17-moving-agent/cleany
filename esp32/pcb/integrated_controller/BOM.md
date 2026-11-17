# 통합 컨트롤러 부품 목록 (Bill of Materials)

통합 컨트롤러 `2.9-draft` 보드 1대에 필요한 PCB 실장품과 별도 조립 자재를 정리한다. 전기적 결선과 커넥터 핀 순서는 [PINMAP.md](PINMAP.md)를 따른다.

## 1. 개요

- **PCB 실장품**: 34개 품목, 총 107개
- **별도 조립 자재**: 7개 품목. 점퍼 캡과 하네스 자재는 PCB 수량에 포함하지 않는다.
- **기준 도면**: [integrated_controller.kicad_sch](integrated_controller.kicad_sch), `2.9-draft`
- **수량 기준**: 보드 1대분, 예비 부품 제외. 치수는 mm 기준이다.

| 파일 | 용도 |
|---|---|
| [BOM_ORDER.csv](BOM_ORDER.csv) | 동일 제조사, MPN과 풋프린트별 수량을 합친 주문용 목록. `PCB` 34행과 `Assembly` 7행 구분 |
| [BOM.csv](BOM.csv) | 개별 Reference 107개의 값, MPN, 풋프린트와 구매 정보 대조 |
| [3D 모델과 라이브러리 출처](libraries/3dmodels/README.md) | 적용 STEP, 이용 조건, 미확보 모델과 EasyEDA 검색 후보 |

`QuantityPerBoard`는 보드 1대분 필요 수량이다. 여러 대를 제작할 때는 제작 대수를 곱한 뒤 공급처의 최소 주문 수량(`MOQ`)과 주문 배수(`OrderMultiple`)를 적용한다. CSV를 스프레드시트로 가져올 때 MPN과 공급처 품번은 텍스트 형식으로 지정해 Molex 품번의 앞자리 `0`을 보존한다.

## 2. 실장 부품 명세

### 2.1 집적회로(IC), 반도체, 인덕터 및 스위치

| Reference | 수량 | 제조사 | 부품 번호 (MPN) | 주요 규격 | 패키지 |
|---|---:|---|---|---|---|
| U1 | 1 | Espressif Systems | [ESP32-S3-WROOM-2-N32R16V](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-WROOM-2-N32R16V/25811280) | Dual-core MCU, 32 MB Octal Flash, 16 MB Octal PSRAM, PCB 안테나 | SMD 모듈 (41패드) |
| U2 | 1 | STMicroelectronics | [USBLC6-2SC6](https://www.digikey.kr/en/products/detail/stmicroelectronics/USBLC6-2SC6/1040559) | 2채널 초저용량 USB ESD 보호기 (Standoff: 5.25 V) | SOT-23-6 |
| U3–U7 | 5 | Texas Instruments | [LMR33640DDDAR](https://www.digikey.kr/en/products/detail/texas-instruments/LMR33640DDDAR/11308833) | 36 V / 4 A 동기식 벅 레귤레이터, 1 MHz | HSOP-8 (PowerPAD) |
| U8 | 1 | Bosch Sensortec | [BNO055](https://www.digikey.kr/en/products/detail/bosch-sensortec/BNO055/6136309) | 9축 절대 방위 센서 (지능형 센서 허브) | 28-LGA (5.2 × 3.8 mm) |
| U9 | 1 | Texas Instruments | [TCA9548ADGSR](https://www.digikey.kr/en/products/detail/texas-instruments/TCA9548ADGSR/25649573) | 8채널 I2C 스위치 / 멀티플렉서 (하드웨어 리셋 구비) | VSSOP-24 (0.5 mm 피치) |
| U10 | 1 | Texas Instruments | [SN74AHCT1G125DCKR](https://www.digikey.kr/en/products/detail/texas-instruments/SN74AHCT1G125DCKR/375375) | 단일 3-State 버퍼 (3.3 V → 5 V TTL 논리 레벨 시프터) | SC-70-5 (SOT-353) |
| D1–D2 | 2 | Comchip Technology | [SS34-HF](https://www.digikey.kr/en/products/detail/comchip-technology/SS34-HF/10279693) | 40 V / 3 A 쇼트키 다이오드, VF 0.55 V @ 3 A (RAW, VBUS → VIN_LOGIC 다이오드 OR) | SMA (DO-214AC) |
| L1–L5 | 5 | Würth Elektronik | [74439346033](https://www.digikey.kr/en/products/detail/w%C3%BCrth-elektronik/74439346033/17734116) | 3.3 µH ±20%, Ir: 8 A, Isat: 7.5 A, DCR: 10.83 mΩ | SMD (6.65 × 6.45 mm) |
| SW1–SW2 | 2 | C&K / Littelfuse | [PTS810SJM250SMTR LFS](https://www.digikey.kr/en/products/detail/c-k/PTS810SJM250SMTR-LFS/4176610) | SPST-NO 택트 스위치 (1.6 N 동작력, 상면 조작형) | 4.2 × 3.2 × 2.5 mm SMT |

### 2.2 커패시터

| Reference | 수량 | 제조사 | 부품 번호 (MPN) | 정격 및 특성 | 패키지 (인치) |
|---|---:|---|---|---|---|
| C5, C12, C19, C26, C33 | 5 | Cal-Chip Electronics, Inc. | [GMC31X5R106K50NT](https://www.digikey.kr/en/products/detail/cal-chip-electronics-inc/GMC31X5R106K50NT/22461301) | 10 µF ±10%, 50 V, X5R | 1206 (3216 Metric), 높이 최대 1.90 mm |
| C6, C13, C20, C27, C34 | 5 | Murata | [GRM188R71H224KAC4D](https://www.digikey.kr/en/products/detail/murata-electronics/GRM188R71H224KAC4D/5797536) | 220 nF ±10%, 50 V, X7R | 0603 (1608 Metric) |
| C7, C14, C21, C28, C35 | 5 | Samsung | [CL10B105MO8NNWC](https://www.digikey.kr/en/products/detail/samsung-electro-mechanics/CL10B105MO8NNWC/5961008) | 1 µF ±20%, 16 V, X7R | 0603 (1608 Metric) |
| C8, C15, C22, C29, C36 | 5 | KYOCERA AVX | [KGM15BR71E104KT](https://www.digikey.kr/en/products/detail/kyocera-avx/KGM15BR71E104KT/1116240) | 100 nF ±10%, 25 V, X7R (BOOT용) | 0603 (1608 Metric) |
| C9–C11, C16–C18, C23–C25, C30–C32, C37–C39 | 15 | TDK | [CGA6P1X7R1C226M250AC](https://www.digikey.kr/en/products/detail/tdk-corporation/CGA6P1X7R1C226M250AC/2443201) | 22 µF ±20%, 16 V, X7R, AEC-Q200 | 1210 (3225 Metric) |
| C1 | 1 | TDK | [C2012X5R1E106K125DB](https://www.digikey.kr/en/products/detail/tdk/C2012X5R1E106K125DB/25855080) | 10 µF ±10%, 25 V, X5R | 0805 (2012 Metric) |
| C2, C3, C40, C43, C44, C45 | 6 | KEMET | [C0603C104K5RACTU](https://www.digikey.kr/en/products/detail/kemet/C0603C104K5RACTU/1465594) | 100 nF ±10%, 50 V, X7R | 0603 (1608 Metric) |
| C4, C42 | 2 | KEMET | [C0603C105K4RACTU](https://www.digikey.kr/en/products/detail/kemet/C0603C105K4RACTU/2199788) | 1 µF ±10%, 16 V, X7R | 0603 (1608 Metric) |
| C41 | 1 | KEMET | [C0603C124K5RACTU](https://www.digikey.kr/en/products/detail/kemet/C0603C124K5RACTU/3736961) | 120 nF ±10%, 50 V, X7R | 0603 (1608 Metric) |

C45는 기존 구매품 `C0603C104K5RACTU`의 여분 1개를 사용한다. 실장 수량에는 포함하지만 추가 발주하지 않는다.

### 2.3 저항

| Reference | 수량 | 제조사 | 부품 번호 (MPN) | 저항값 및 정격 | 패키지 (인치) |
|---|---:|---|---|---|---|
| R2, R4, R5, R7, R9, R11, R22 | 7 | Vishay Dale | [CRCW0603100KFKEA](https://www.digikey.kr/en/products/detail/vishay-dale/CRCW0603100KFKEA/1174896) | 100 kΩ ±1%, 0.1 W, 100 ppm/K | 0603 (1608 Metric) |
| R3 | 1 | Panasonic | [ERJ-3EKF4322V](https://www.digikey.kr/en/products/detail/panasonic-industry/ERJ-3EKF4322V/198386) | 43.2 kΩ ±1%, 0.1 W (3.3V FB 하단) | 0603 (1608 Metric) |
| R6, R8, R10, R12 | 4 | YAGEO | [RC0603FR-0724K9L](https://www.digikey.kr/en/products/detail/yageo/RC0603FR-0724K9L/727080) | 24.9 kΩ ±1%, 0.1 W (5.0V FB 하단) | 0603 (1608 Metric) |
| R1, R13–R14, R19, R21 | 5 | Panasonic | [ERJ-3EKF1002V](https://www.digikey.kr/en/products/detail/panasonic-electronic-components/ERJ-3EKF1002V/196066) | 10 kΩ ±1%, 0.1 W | 0603 (1608 Metric) |
| R15–R16, R17–R18 | 4 | YAGEO | [RC0603FR-074K7L](https://www.digikey.kr/en/products/detail/yageo/RC0603FR-074K7L/727212) | 4.7 kΩ ±1%, 0.1 W (I2C 풀업) | 0603 (1608 Metric) |
| R20 | 1 | Vishay Dale | [CRCW0603330RFKEA](https://www.digikey.com/en/products/detail/vishay-dale/CRCW0603330RFKEA/1174611) | 330 Ω ±1%, 0.1 W (표준 동작, LED 데이터 댐핑) | 0603 (1608 Metric) |
| R23 | 1 | Vishay Dale | CRCW060322K0FKEA | 22 kΩ ±1%, 0.1 W, 100 ppm/K (배터리 분압 하단) | 0603 (1608 Metric) |

### 2.4 커넥터

| Reference | 수량 | 제조사 | 부품 번호 (MPN) | 규격 및 정격 | 장착 형식 |
|---|---:|---|---|---|---|
| J1 | 1 | Generic | **MPN 미정** | 수직형 XT60 수형 2P (Pin 1: RAW +, Pin 2: GND −), 실물 대조 필요 | 관통형 (TH) |
| J2 | 1 | On Shore Technology Inc. | [USB-B1HSB6](https://www.digikey.kr/en/products/detail/on-shore-technology-inc/USB-B1HSB6/2677744) | USB Type-B 리셉터클, 신호핀 4개, 수평 직각 | 신호핀 4개와 실드 고정핀 2개 모두 THT |
| J3 | 1 | Phoenix Contact | [1989450](https://www.digikey.kr/en/products/detail/phoenix-contact/1989450/2513864) | 3P 스크루 단자대 (5.00 mm 피치, 17.5 A 정격) | 관통형 (TH) |
| J4–J6 | 3 | Phoenix Contact | [1935161](https://www.digikey.kr/en/products/detail/phoenix-contact/1935161/568614) | 2P 스크루 단자대 (5.00 mm 피치, 17.5 A 정격) | 관통형 (TH) |
| SW3 | 1 | Molex | [0022057025 / 5268-02A](https://www.molex.com/en-us/products/series-chart/5268) | 외부 3.3 V EN 스위치용 Mini-SPOX 2P 수평 직각 헤더 (2.50 mm 피치) | 관통형 (TH) |
| J7–J10 | 4 | Molex | [0022057035 / 5268-03A](https://www.digikey.kr/en/products/detail/molex/0022057035/403342) | Mini-SPOX 3P 직각 수평 웨이퍼 헤더 (2.50 mm 피치) | 관통형 (TH) |
| J11–J14 | 4 | Molex | [0022057045 / 5268-04A](https://www.digikey.kr/en/products/detail/molex/0022057045/403343) | Mini-SPOX 4P 직각 수평 웨이퍼 헤더 (2.50 mm 피치) | 관통형 (TH) |
| JP1 | 1 | Sullins Connector Solutions | [PREC002SAAN-RC](https://www.digikey.kr/en/products/detail/sullins-connector-solutions/PREC002SAAN-RC/2774852) | 2P 수직 핀 헤더 (2.54 mm 피치, 3 A), USB 급전 경로 분리용 | 관통형 (TH) |
| J15–J18 | 4 | Assmann WSW | [A-2004-3-4-LP-N-R](https://www.digikey.kr/en/products/detail/assmann-wsw-components/A-2004-3-4-LP-N-R/2183632) | 4P4C 비차폐 수평 모듈러 잭 (RJ9/RJ22) | 관통형 (TH) |

R4는 U3의 EN을 GND로 당기는 100 kΩ 풀다운이며, 기존 100 kΩ 0603 저항과 같은 발주품을 사용한다. SW3는 PCB 실장 택트 스위치가 아니라 외부 접점용 2핀 헤더다.

## 3. PCB 풋프린트 규격

제작 조건과 KiCad DRC 설정은 [DESIGN_RULES.md](DESIGN_RULES.md)를 따른다.

U1은 표준 풋프린트 `RF_Module:ESP32-S3-WROOM-2`를 사용한다.

프로젝트 전용 풋프린트 라이브러리(`libraries/footprints/Integrated_Controller.pretty/`) 사양은 다음과 같다.

| 구분 | 적용 풋프린트 | 도면 반영 치수 및 설계 기준 |
|---|---|---|
| L1–L5 | `Wuerth_74439346033_6.65x6.45mm` | 패드 크기: 5.50 × 1.45 mm, 패드 중심 간격: 4.00 mm |
| J15–J18 | `RJ9_Assmann_A-2004-3-4-LP-N-R_Horizontal` | 접점 홀: Ø0.90 mm, 고정 페그 홀: Ø3.20 mm (좌우 2개소) |
| J1 | `XT60_Generic_Vertical_1x02_P7.20mm_Pin1Positive` | 핀 피치: 7.20 mm, 드릴 직경: Ø4.50 mm (Pin 1: 사각/RAW, Pin 2: 모따기/GND) |

## 4. 하네스 및 체결 자재 명세 (별도 발주품)

PCB 실장품과 함께 주문할 점퍼 캡과 하네스 자재다. `BOM_ORDER.csv`의 `Scope=Assembly` 행에 대응한다. 크림프 터미널 28개는 3P 하우징 4개와 4P 하우징 4개에 필요한 기본 수량이며, 작업 예비분은 별도로 더한다. SW3의 외부 스위치, 2P 하우징과 접점은 별도 하네스 세트로 선정한다.

| 용도 | 부품 번호 | 사양 | 소요 수량 |
|---|---|---|---:|
| 모터 신호 하네스 (J7–J10) | Molex `0050375033` (`5264-03`) | Mini-SPOX 3P 크림프 하우징 (2.50 mm 피치) | 4 EA |
| 엔코더 하네스 (J11–J14) | Molex `0050375043` (`5264-04`) | Mini-SPOX 4P 크림프 하우징 (2.50 mm 피치) | 4 EA |
| 공통 크림프 터미널 | Molex `0008701040` (`08-70-1040`) | 5263 계열 압착 핀, AWG 22–28 대응 | 28 EA |
| ToF 센서 버스 하네스 (J15–J18) | 표준 4P4C 플러그 | 4심 플랫 케이블 (양단 1:1 직결 결선) | 4 SET |
| USB 급전 점퍼 (JP1) | Sullins [QPC02SXGN-RC](https://www.digikey.kr/en/products/detail/sullins-connector-solutions/QPC02SXGN-RC/2618262) | 2P 쇼트 점퍼 (2.54 mm 피치, 3 A) | 1 EA |
| 전원 주 케이블 (J1) | 표준 XT60 암형(Female) 커넥터 | 정격 전선 연결 (Pin 1: RAW +, Pin 2: GND −) | 1 SET |
| 3.3 V EN 스위치 하네스 (SW3) | 미정 | 외부 접점 스위치와 호환 2P Mini-SPOX 하네스 | 1 SET |

## 5. 발주 전 확인

| 대상 | 확인할 내용 |
|---|---|
| J1 | 정확한 MPN 선정 후 임시 풋프린트의 핀 피치 7.20 mm, 홀 4.50 mm, 수직 방향, 성별과 극성을 실물과 대조 |
| U1 | 개발 보드가 아닌 `ESP32-S3-WROOM-2-N32R16V` 모듈 주문. DigiKey CT 품번 `1965-ESP32-S3-WROOM-2-N32R16VCT-ND`, [디바이스마트 상품](https://www.devicemart.co.kr/goods/view?no=16095449)의 정확한 모델과 수량 확인 |
| C5, C12, C19, C26, C33 | DigiKey CT 품번 `4713-GMC31X5R106K50NTCT-ND`, 최소 1개, 주문 배수 1개. 보드 1대분은 5개이며 [디바이스마트](https://www.devicemart.co.kr/goods/view?no=16138686)에서도 CT 포장을 선택한다. 입력 전압의 DC 바이어스에서 유효 정전 용량 확인 |
| SW3 | PCB에는 5268-02A 헤더 실장. 외부 스위치의 동작 방식과 VIN_LOGIC 전압에 맞는 접점 정격, 하우징, 접점 및 케이블을 선정하고 구매처 확인 |
| 별도 하네스 | 케이블 길이, 전선 규격과 실제 상대 커넥터 확인. 4P4C 케이블은 양단 핀 번호 기준 1:1 연속성 확인 |

`BOM.csv`의 재고 수량은 `StockChecked`에 기록된 시점의 조회값이다. `MOQ`와 `OrderMultiple`이 `결제 시 확인`인 항목은 발주할 때 확인한다. 구매 정보의 빈 칸은 미확인 항목이다.

## 6. 도면 및 기술 문서 참조

- Espressif Systems: [ESP32-S3-WROOM-2 Datasheet v1.7](https://documentation.espressif.com/esp32-s3-wroom-2_datasheet_en.html), Table 1-1, Table 3-1, Figure 11-1
- Würth Elektronik: Component Drawing 74439346033 (Rev. 001.000)
- Assmann WSW Components: Drawing ASS 7041 CO (Rev. 00, 2015-10-28)
- On Shore Technology: [USB-B1HSXX drawing](https://www.on-shore.com/wp-content/uploads/USB-B1HSXX.pdf), p. 1
- C&K Switches: PTS810 Series Tact Switch Product Drawing
- Molex: Sales Drawing 52680001-SD (Rev. B, 2023-03-09)
- Phoenix Contact: Technical Data Sheet PT 1,5/ 3-5,0-H BK (1989450) / PT 1,5/ 2-5,0-H (1935161)
- Cal-Chip Electronics: [GMC Series](https://calchip.com/wp-content/uploads/2025/05/GMC-Series-3.pdf), 부품 번호 체계 p. 2, 권장 랜드 치수 p. 19
