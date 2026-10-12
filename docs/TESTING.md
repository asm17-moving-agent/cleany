# 테스트 가이드

기준 환경은 Humble/Fortress다. 명령은 저장소 루트에서 실행한다.
변경 범위의 작은 검사부터 선택하며 Make target의 실제 동작은 [Makefile](../Makefile)을 따른다.

## 검사 선택

```mermaid
flowchart TB
    Change{"바뀐 경계"}
    Change -->|"순수 logic / 도구"| Unit["해당 core / host test"]
    Change -->|"ROS type / manifest / launch"| Package["관련 package build + test"]
    Change -->|"여러 package 연결"| ROS["합성 ROS runtime 검사"]
    Change -->|"센서 / 주행 / 조작"| Sim["해당 simulator runtime 검사"]
    Change -->|"UI"| UI["served app + refresh / reconnect / replay"]
    Change -->|"문서"| Docs["링크 / 경로 / Make dry-run / KB 무결성"]
    Unit --> Evidence["실행 조건과 결과 원본 보존"]
    Package --> Evidence
    ROS --> Evidence
    Sim --> Evidence
    UI --> Evidence
    Docs --> Evidence
```

구체 명령은 아래 표에서 선택한다. Simulator·mock 통과를 실물 검증으로 승격하지 않는다.

| 변경 범위 | 명령 | 확인 범위 |
|---|---|---|
| 전체 ROS workspace | `make test` | build, colcon package test와 출력 정리 도구 |
| Mission·Gateway core | `make test-mission-core` | ROS 없는 lifecycle·영속성·중복·재전송 |
| Mission ROS 설치·연동 | `make test-mission-runtime` | 관련 package build/test |
| Navigation core·plugin | `make test-navigation` | Python core와 C++ plugin |
| 합성 Navigation runtime | `make test-navigation-runtime` | ROS lifecycle·goal·취소·sensor 단절 |
| Gazebo·Navigation | `make test-gazebo` | package 검사와 C++ plugin; 선택 runtime은 제외 |
| Gazebo sensor runtime | `make test-gazebo-nav-runtime` | 실제 simulator의 sensor·odom·TF |
| 저장 지도 목표 회귀 | `make test-gazebo-navigation-goals` | 장거리·chair-gap 주행; 새 출력 디렉터리 |
| 시설 목표 생성 | `make test-gazebo-facility-tools` | simulator 없는 ID·여유·제거 의자 처리 |
| MuJoCo | `make test-mujoco` | scene loader; 모든 조작 runtime을 뜻하지 않음 |
| Hand-eye 경계 | `make test-handeye` | description·MuJoCo·MoveIt·calibration |
| Base ROS | `make test-base` | interface·driver·odometry; mock MCU |
| ESP32 준비·motor core | `make test-micro-ros-setup test-motor-core` | Python 준비 도구와 장치 없는 C++ |
| Monitor | `make test-dev-monitor` | 수집·저장·HTTP와 UI 데이터 경계 |
| Telemetry | `make test-telemetry` | odom relay |
| 시나리오 보조 코드 | `make test-scenario-tools` | fixture·process 소유권·결과 판정 |
| 빌드 정리 도구 | `make test-tools` | 선택 출력 삭제와 보호 경로 거부 |

Fedora에서는 `make test-scenario-tools` 등 native target도 Humble Distrobox에서 실행한다.
`test-micro-ros-setup`, `test-motor-core`, `test-tools`는 Makefile의 `BASE_EXEC`가 환경을 선택한다.

## 선택 runtime과 UI

Gazebo package의 `--run-sim-runtime`, `--run-evaluation-tests`와
Mission core의 `CLEANY_RUN_ROS_TESTS=1`은 별도 선택 검사다. skip은 성공 증거가 아니다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
ROS_DOMAIN_ID=82 CLEANY_RUN_ROS_TESTS=1 make test-mission-core
```

Monitor smoke는 Monitor가 같은 domain에서 먼저 실행 중이어야 한다.

```bash
ROS_DOMAIN_ID=187 python3 tests/dev_monitor/smoke.py --output /absolute/new-run
# frontend의 현재 Playwright 설정과 package scripts:
cat ros2_ws/src/cleany_dev_monitor/web/package.json
```

UI 변경은 실제 served app의 새로고침·재연결·기록 재생도 확인한다.
Playwright browser가 없으면 configured browser를 설치한 뒤 다시 실행한다.
Mission Gateway smoke와 navigation smoke는 `tests/mission_runtime/`, `tests/dev_monitor/`에 있다.
실제 Backend·ROS·Gazebo 입력 조건을 갖춘 경우에만 명시적으로 실행한다.

## 반복 시나리오

[시나리오 실행법과 판정](../tests/scenario_validation/README.md)을 따른다.
단일 smoke 후 반복 범위를 늘리고 sensor freshness, 정지·도착·deadline 기준을 유지한다.
fixture와 overlay, ROS domain, Gazebo partition, DB, 출력 디렉터리를 실행별로 분리한다.

성공은 프로세스 exit나 action terminal만으로 판정하지 않는다. 요청 좌석, terminal 결과,
도착 근거, 최종 정지, 반복 완전성과 child cleanup을 함께 확인한다.
원래 campaign과 fresh 재시험은 합산하지 않는다.

## 결과와 증거

| 증거 | 의미 |
|---|---|
| 소스·manifest·단위 검사 | 구현과 검사된 입력의 계약 |
| mock / 합성 ROS | 합성 backend의 lifecycle과 메시지 경계 |
| Gazebo·MuJoCo runtime | 해당 simulator·입력·버전의 동작 |
| 장치 통신 | 실제 USB·UART·DDS 등 확인한 계층 |
| 물리 시험 | 실제 sensor·actuator·정지·시나리오 결과 |

결과에는 code commit와 dirty 상태, 환경·overlay, 입력·config·asset hash,
요청 범위와 실제 완료 범위, 실패·skip·중단, 로그와 출력 위치를 남긴다.
CTest wrapper와 pytest 집계가 겹치면 합산하지 않는다.
실행 원본은 `CLEANY_DATA_ROOT/runs/`, 과거 보고서는 [근거 색인](EVIDENCE.md)에 둔다.

## 문서 변경 검사

내부 파일 링크·anchor, 코드·설정·launch 경로, Make target의 존재와 native dry-run을 확인한다.
실제 로봇을 움직이거나 새 calibration을 수행하는 명령은 문서 검증만을 위해 실행하지 않는다.
KB 변경에는 KB의 `kb-quality-checks`를 사용하며 통과와 내용 승인을 구분한다.
