# VL53L7CH USB 거리 센서 펌웨어

독립 ESP32-S3 DevKitC-1(플래시 16 MiB)이 ST VL53L7CH를 I²C로 읽고, 구역별 거리와 품질을 내장 USB CDC/JTAG로 전송한다. PlatformIO가 C++ 애플리케이션을 빌드한다. Arduino-ESP32의 `Wire`와 `Serial`을 사용하고, 센서 초기화와 측정은 ST C 드라이버 `VL53LMZ_2.0.10`을 직접 호출한다. 드라이버 버전과 부분 읽기 오류 수정은 [UPSTREAM.txt](lib/st-vl53lmz/UPSTREAM.txt)에 기록돼 있다.

| 인터페이스 | 설정 |
|---|---|
| 센서 I²C | SDA GPIO8, SCL GPIO9, 7비트 주소 `0x29`, 기본 400 kHz |
| 호스트 USB | 내장 USB D− GPIO19, D+ GPIO20. 명령과 응답은 줄바꿈으로 구분 |
| 부팅 기본값 | 8×8, 10 Hz, 연속 측정, 구역당 타깃 1개 |
| ROS 연결 후 | `tof_driver`가 `stop` 응답을 받은 뒤 `cnh off`, `config 64 15 400000`, `start`를 순서대로 전송 |

센서 전원과 LPn, INT, I2C_RST 배선은 당시 실물 기록에서 확인한 범위 밖이다. 거리 배열은 센서 고유 순서이며 거리 단위는 mm다. 타깃이 검출되고 ST 상태가 5 또는 9인 구역만 `valid=true`로 표시한다. [ROS 인터페이스](../../ros2_ws/src/cleany_tof/docs/interfaces.md)

## 빌드와 업로드

저장소 루트에서 실행한다. 호스트에 Python 3과 venv, Make가 필요하다.

```bash
python3 -m venv esp32/tof_vl53l7ch/.venv
source esp32/tof_vl53l7ch/.venv/bin/activate
python -m pip install -r esp32/tof_vl53l7ch/requirements.txt
make test-tof-firmware
make tof-firmware-build
```

부분 읽기 회귀 검사에는 ASan과 UBSan을 지원하는 C 컴파일러가 필요하다. 빌드 산출물은 `CLEANY_BUILD_ROOT/tof-firmware/pio/esp32s3/`에 생성된다. 업로드 전 ToF 보드의 USB by-id 경로를 확인하고, 같은 포트를 사용하는 ROS 드라이버나 시리얼 모니터를 닫는다.

```bash
ls -l /dev/serial/by-id/
make tof-firmware-upload CLEANY_TOF_PORT="/dev/serial/by-id/<ToF 보드 ID>" CONFIRM_UPLOAD=1
```

업로드 후 USB에서 `info`를 보내면 펌웨어 버전, 핀, 주소, 구역 수, 주파수를 확인할 수 있다. `stats`는 누적 I²C 오류, 읽기 오류, USB 드롭을 반환한다. ROS에서 확인할 때는 [ToF 실행 절차](../../docs/RUNNING.md#tof-근접-보호와-책상-접근)를 따라 `tof/frame`을 수신한다.

## 실물 기록과 재생

`CLEANY_DATA_ROOT` 아래에 2026-10-25 단독 보드의 검증 자료가 있다.

- 보고서와 배선·기능 시험 결과는 `runs/hardware/2026-10-25_tof/reports/vl53l7ch-20261025/outputs/vl53l7ch-esp32/VALIDATION.md`에 있다.
- 원본 JSONL, 시험본 해시 목록, 플래시 검증 로그는 `runs/hardware/2026-10-25_tof/vl53l7ch-20261025/outputs/vl53l7ch-esp32/evidence/`에 있다.
- 당시 업로드한 바이너리와 플래시 오프셋은 `datasets/reference/tof/vl53l7ch-20261025/outputs/vl53l7ch-esp32/bin/`에 있다.

8×8, 15 Hz로 설정한 120초 측정 구간에서 1,816프레임(15.1496 Hz)을 수신했고 해당 구간의 순번 단절, I²C 읽기 오류, USB 드롭은 모두 0건이었다. 전체 로그에는 마지막 통계 응답을 기다리는 동안 수신한 프레임 한 개가 더 있다. 이 기록은 단독 보드에서 측정한 값이다. [ROS 파서 재생 절차](../../docs/TESTING.md#tof와-supervisor-통합)
