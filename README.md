# Cleany 로봇 소프트웨어

ROS 2 주행과 매핑, 센서 관측, 물체 조작 시뮬레이션, ESP32 구동을 개발합니다.

**[문서 홈](docs/index.md) · [브라우저로 문서 읽기](docs/index.html) · [개발환경 설치](docs/DEVELOPMENT_SETUP.md) · [팀 KB](docs/cleany-docs/README.md)**


## 바로 시작하기

| 목표 | 안내 |
|---|---|
| 처음 빌드·실행 | [개발환경](docs/DEVELOPMENT_SETUP.md) |
| 센서 입력과 인식 확인 | [센서 가이드](docs/how-to/sensors.md) |
| 물체 관찰·계획·조작 | [조작 가이드](docs/how-to/manipulation.md) |
| MCU와 휠 구동 확인 | [베이스 가이드](docs/how-to/base.md) |
| 휴대폰 목표 주행 | [화면으로 따라 하기](docs/how-to/phone-navigation.md) |
| 하드웨어로 지도 만들기 | [매핑 실행](ros2_ws/src/cleany_bringup/docs/hardware-mapping.md) |
| 시스템과 ROS 연결 이해 | [연동 구조](docs/INTEGRATION.md), [Navigation 구성](ros2_ws/src/cleany_navigation/README.md) |
| 시뮬레이터 실행 | [시뮬레이션](docs/how-to/simulation.md) |
| 기록과 상태 관측 | [Logger·Monitor](docs/how-to/monitoring.md) |
| 코드 변경 검증 | [검증 안내](docs/TESTING.md) |

기준 환경은 Ubuntu 22.04, ROS 2 Humble, Python 3.10, Gazebo Fortress입니다. 저장소 루트의 Humble 터미널에서 시뮬레이션을 시작합니다.

```bash
git submodule update --init --recursive docs/cleany-docs
source /opt/ros/humble/setup.bash
make deps-gazebo
make test-gazebo
make sim-gazebo
```

전체 workspace는 `make deps`, `make build`, `make test`를 사용합니다. 명령의 범위와 준비 조건은 [개발환경](docs/DEVELOPMENT_SETUP.md)과 [검증 안내](docs/TESTING.md)를 따릅니다. 시뮬레이션과 실물 동작은 구분해서 확인합니다.

## 코드와 문서

| 위치 | 역할 |
|---|---|
| `ros2_ws/src/` | ROS 패키지, core, launch와 설정 |
| `esp32/` | MCU 펌웨어, 빌드·업로드, 배선 |
| `tests/`, `tools/` | 통합 검사와 개발·분석 도구 |
| `docs/` | 작업 가이드와 시스템 설명 |
| 각 패키지 `docs/` | 인터페이스, 설정, 단위와 동작 조건 |
| `docs/cleany-docs/` | 제품 범위와 설계 결정을 관리하는 별도 KB |

문서 사이트는 `docs/index.html`을 브라우저에서 열면 됩니다. Markdown을 수정한 뒤 `python3 tools/docs/build_site.py`로 갱신하며, `--check`로 원본과 일치하는지 확인합니다. 빌드·실행 원본의 로컬 경로는 팀 공유 위치를 의미하지 않습니다.
