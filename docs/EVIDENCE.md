# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

## PR 65 main 충돌 해소 검증 (2026-11-02)

기존 #65 `db4ebed`에 main `c50c440`을 병합했다. `.gitignore`와 보정, 개발환경,
실행, ROS 색인 문서 4개의 충돌에서 실물 제어와 디스플레이, ToF 내용을 유지했다.
`/output/` 규칙이 디스플레이 산출물도 포함하므로 중복된 하위 경로 규칙은 제거했다.
main 대비 기능 변경 81개 파일과 Makefile 추가 명령은 기존 #65와 동일하다.

macOS Python 3.12의 별도 venv에서 가짜 버스와 합성 설정으로 제어 및 호스트 도구
107개, 상위 상태와 복구 52개, ToF core 91개 검사가 통과했다. CI 도구 9개도
통과했으며 제어 및 호스트 도구 107개에 포함된 검사다. 최초 누락된 Python 의존성은
검증 venv에 설치했다. SDK 가상 시리얼 1개는 macOS PTY의 1 Mbps ioctl이 지원되지
않아 실패했으며, 해당 검사를 제외한 로컬 재실행 결과를 위에 기록했다. 테스트는 유지했다.
Jetson 이름 해석이 되지 않아 native ROS 빌드, C++와 ROS 연결 검사는 재실행하지 않았다.
충돌 표식, 문서 링크, output 제외와 기능 변경 보존을 별도로 확인했다.
원본 로그와 병합 검사는 `/Users/hyeon/workspace/cleany-data/reports/pr65-main-20261102/`에 보관한다.

### 하드웨어 매핑 병합 후 재검증

main에 #70이 추가된 뒤 `102a63e`에서 main `32d2d9e`를 반영했다. 개발환경,
실행과 ROS 색인 문서 3개의 충돌은 양쪽 절을 모두 유지해 해결했다.

직전 GitHub CI run `41376624141`은 Feetech SDK 1.0.0 설치에는 성공했지만
rosdep이 `feetech-servo-sdk==1.0.0`의 설치 완료를 인식하지 못해 의존성 이미지
단계에서 실패했다. `102a63e`에 이 절과 함께 반영한 수정은 rosdep에 패키지
이름을 지정하고 CI constraints로 버전 1.0.0을 유지한다. macOS Python 3.12,
rosdep 0.27.0과 실제 설치된 SDK로 기존 규칙의 감지 실패와 수정 후 성공을
재현했으며, constraints를 적용한 pip dry-run도 같은 버전을 확인했다.

합성 입력과 가짜 transport를 사용한 매핑과 조종 Python 검사 60개, Node 입력
검사 17개와 `make test-ci-tools` 9개가 통과했다. Make의 login shell이 macOS
시스템 Python을 선택해 pytest를 찾지 못한 뒤, 동일 Python 검사 목록을 venv에서
직접 실행했다. Linux `/proc` 기반 PID 재사용 검사는 macOS에서 실패했으며,
해당 1개를 제외한 재실행 결과가 60개다. 검사 코드는 변경하지 않았다.
Jetson 이름 해석 실패와 로컬 Docker daemon 부재로 native ROS 빌드와 전체
workspace 검사는 재실행하지 않았다. CI 결과와 실물 동작 검증은 별도다.

원본 실패, 재실행 로그와 rosdep 감지 결과는
`/Users/hyeon/workspace/cleany-data/reports/pr65-main-20261102/recheck/`에 보관한다.

## 과거 생성 보고서 보관

날짜별 MuJoCo 성능, 수거 시험 보고서와 SCRUM-486 검증 JSON은
`CLEANY_DATA_ROOT/reports/evidence/` 아래 원래 상대 경로로 보관합니다.
새 실행 산출물은 Git에서 제외하고 commit, 환경, 입력, 실패와 원본 위치를 남깁니다.

## Jetson 제어 기반 분리 검증

`3dea85d`에 미커밋 정리 패치를 적용한 소스를 Ubuntu 22.04 ARM64,
ROS 2 Humble과 MoveIt 2.5.9에서 별도 빌드 경로로 검사했다.
`make build-jetson-control`은 6개 패키지 빌드, `make test-jetson-control`은
163개 검사를 통과했다. 실패와 skip은 없었다. 입력은 저장소의 고정 보정,
합성 상태와 가상 시리얼이며 실물 장치를 열지 않았다.
환경, 입력 소스를 식별하는 `changes.patch`, 로그와 JUnit 원본은
`/home/cleany/cleany-data/reports/pr-a-trim-20261029T185419Z/`에 보관한다.

### PR 65 미사용 설정 정리 검증 (2026-10-30)

갱신된 PR 64를 병합한 `197de90`에 이 절과 함께 반영한 정리 패치를 적용했다.
TCP와 중복 손목 보정 3개, 기본 Xacro가 사용하지 않는 미리보기 인자와
matplotlib 의존성을 제거했다. 기존 미리보기와 새 미리보기의 URDF hash는 같다.
Jetson Ubuntu 22.04 ARM64, Humble과 MoveIt 2.5.9의 새 경로에 6개 패키지를
빌드했고 삭제한 설정의 부재와 설치된 설정, launch 5개의 소스 일치를 확인했다.

`make test-jetson-control` 163개, `make test-jetson-controller` 21개와 C++ 드라이버
11개가 통과했다. 기존 제어 검사는 모두 유지했으며 실패와 skip은 없다.
가상 시리얼과 합성 입력만 사용했고 실물 모터와 전체 workspace 검사는 실행하지 않았다.
기존 Xacro 경고와 C++ 테스트의 write 반환값 경고는 유지된다.
소스 패치, 환경, 로그, JUnit과 설치 및 URDF 비교 근거는
`/home/cleany/cleany-data/reports/pr64-65-cleanup-20261029T233007Z/pr65/`에 보관한다.

## Nav2 복구 자료의 저장소 외부 보관

PR #73의 복구 자료 `recovered/nav2/`는 구현 코드와 분리해 데이터 루트의
`archives/pr73-recovery-cleanup-20261102-c978e838/recovered/nav2/`로 옮겼다.
이동 전 Git 기준은 `c978e838e9f9cca8afb1bb386fa7c960c47bfdb4`이며, 원본 출처와
검증 기록은 보관 디렉터리의 `README.md`, `INDEX.json`, `provenance/`에 있다.
활성 MPPI 구현과 설정은 `ros2_ws/src/cleany_navigation/`에 유지한다.

## PR #73 snapshot 다운로드 지연

`25b8eca28ad7f493087d68464385414203715a32`의
[CI 41379752233](https://github.com/asm17-moving-agent/cleany/actions/runs/41379752233)은
Ubuntu snapshot의 반복된 HTTP 502와 503으로 이미지 생성에서 33분 이상 경과해 중단했다.
코드 빌드와 테스트는 시작하지 않았다. 의존성 명령 재시도뿐 아니라 APT의 파일별 재시도가 누적됐다.
원본은 데이터 루트의 `reports/ci-dependency-pinning-20261102/pr73-ci-41379752233.log`다.

서명이 검증된 snapshot index의 SHA256과 크기를 유지하면서 동일 pool 파일을 archive 미러에서
먼저 받는 다운로드 도구를 추가했다. SHA256 또는 크기가 다르면 cache에 넣지 않으며 snapshot으로
돌아간다. 미러 index를 읽거나 버전을 변경하지 않는다. 설치 명령 다운로드 5분과 이미지 생성
15분 상한을 추가하고 APT 자체 재시도 1회, 명령 실행 최대 2회로 줄였다.
다운로드 도구와 기존 CI 도구의 자동 검사 24건이 통과했다. 고정 amd64 컨테이너에서
이전 실패 패키지 `libxrandr-dev`, `mesa-vulkan-drivers`, `python3-pip-whl`,
`python3-setuptools-whl`을 미러 다운로드와 SHA256 검증으로 재설치했다.
이후 버전 검사에서 Python 36개와 공통 ROS 311개가 계속 일치했다.
설치 로그는 같은 데이터 루트의 `pr73-combined/pr73-mirror-install.log`다.

## PR #73 통합 빌드의 병렬 제한과 MPPI 계약 검사

2026-11-02에 PR head `25b8eca28ad7f493087d68464385414203715a32`와 아래 수정 내용을
고정 의존성 amd64 이미지 `61625b7ef43b330932899ca41fa455e0570962dbc1549dba6ae221a583b160d0`에서 검증했다.
컨테이너는 CPU 3개 분량과 메모리 8 GiB로 제한했다. 첫 빌드에서는 colcon이
`CMAKE_BUILD_PARALLEL_LEVEL=2`에도 `-j12 -l12`를 주입해 MPPI 컴파일러 5개가 OOM으로 종료됐다.
`MAKEFLAGS=-j2 -l2`를 함께 전달한 뒤 28개 패키지가 빌드됐고 추가 OOM은 없었다.

통합 검사에서 이전 GoalDirectedController와 배포판 MPPI를 기대하던 assertion 6건이 실패했다.
기대 플러그인을 현재 Cleany MPPI로 갱신하고 Omni, 횡이동, footprint 비용과 Smac Lattice 조건을 확인했다.
검사를 제거하거나 제어 설정을 완화하지 않았다. 관련 검사 22건과 전체 2820 reported tests가
통과했다(errors/failures 0, 기존 skip 32). 별도 Nav2 synthetic ROS 실행 검사 3건,
Gazebo 센서/TF 실행 검사 1건과 motor core 검사도 통과했다.
실제 로봇 검증은 수행하지 않았다.

원본 로그는 데이터 루트의 `reports/ci-dependency-pinning-20261102/pr73-combined/`에 보관한다.

## CI 의존성의 Jetson 버전 기준

PR #73의 `20dae9af760e3212fbec8c0b5476ab73ff928d90`에서 실행한
[CI 41377740118](https://github.com/asm17-moving-agent/cleany/actions/runs/41377740118)은
이미지 생성 중 Ubuntu snapshot 서버가 패키지 4개에 HTTP 502를 반환해 실패했다.
`ros-humble-mujoco-ros2-control` 설치 단계에서 중단됐으며 코드 빌드와 테스트에는 도달하지 않았다.
고정 snapshot과 Python constraints를 유지한 채 APT 다운로드 재시도 및 의존성 준비 명령의 제한된 재시도를 추가했다.
원본은 데이터 루트의 `reports/ci-dependency-pinning-20261102/pr73-ci-41377740118-failed.log`다.

2026-11-02에 `jetson-orin`의 Ubuntu 22.04 arm64 system Python과 `dpkg-query -W`를 읽었다.
최종 검증 기준은 `c50c440540a7c5c4c0736635d9d122b4736b2a47`이며 변경은 `feat/ci-dependency-pinning` 작업 트리에서 검증했다.
실물의 패키지 설치와 설정은 변경하지 않았다.

ROS `2026-08-07` snapshot의 amd64와 arm64 package index 모두 Jetson의 ROS 패키지 388개와 upstream 버전이 일치했다.
아키텍처별 Debian 빌드 시각은 비교에서 제외했다. MuJoCo `3.11.0`과 google-genai `2.18.0`의 Python 의존성 36개를 조회해 constraints로 보존했다.
Ubuntu 저장소는 `20260807T000000Z` snapshot으로 고정했다. 이는 Jetson 전체 OS 패키지나 JetPack의 동일성을 뜻하지 않는다.

로컬 arm64 이미지 생성은 해당 ROS snapshot에 `ros-humble-ros-gz-sim`이 없어 실패했다.
같은 snapshot의 amd64에는 `0.244.25-1jammy.20260726.122251`이 있어 실제 CI와 같은 amd64에서 검증한다.
첫 amd64 생성은 베이스에 남은 TF2 `0.25.20` 등과 Jetson의 `0.25.22` 차이를 버전 검사에서 감지해 실패했다.
`rosdep install`이 이미 설치된 패키지를 갱신하지 않아, workspace 설치 전에 베이스 ROS 패키지를 snapshot으로 동기화하도록 수정했다.
재생성은 저장소의 SHA256과 대조한 APT 파일 캐시를 사용하며 workspace 빌드와 테스트 결과는 재사용하지 않는다.
첫 workspace 빌드는 BT.CPP `4.9.1`의 CMake export가 `lib/`에서 라이브러리를 찾지만 실제 파일은 `lib/x86_64-linux-gnu/`에 있어 실패했다.
같은 바이너리에 대한 호환 링크를 추가한 뒤 `cleany_manipulation_bt` 단독 빌드를 통과했다.
최종 검증은 amd64 Ubuntu 22.04, Python 3.10.12 컨테이너에서 새 workspace 빌드 디렉터리와 앞선 시도의 컴파일 캐시를 사용했다. 모든 테스트는 다시 실행했다.

초기 `326c237c` 기준의 24개 패키지 빌드와 2,561개 검사도 통과했다.
검증 도중 병합된 ToF와 Supervisor를 포함하도록 `c50c4405`에 동기화한 뒤 아래 전체 검사를 다시 실행했다.

| 검사 | 결과 |
|---|---|
| 의존성 이미지 생성과 버전 검사 | Python 36개, 공통 ROS 311개 일치 |
| `make test-ci-tools` | 13개 통과 |
| `make test`: 전체 빌드 | 27개 패키지 통과 |
| `make test`: workspace 검사 | 2,720개, 오류와 실패 0개, skip 32개 |
| `make test-motor-core` | 통과 |
| 합성 Navigation runtime | 3개 통과 |
| Gazebo sensor/TF runtime | 1개 통과 |

설치 버전, 이미지 ID와 dependency key는 원본의 `amd64-current/image.json`, `amd64-current/dpkg-packages.txt`, `amd64-current/pip-packages.txt`에 남겼다.

원본 Jetson 환경, package index, 이미지 빌드와 검사 로그는 데이터 루트의
`reports/ci-dependency-pinning-20261102/`에 보관한다.

## Jetson 실물 마운트와 카메라 보정

2026-10-21에 `jetson-orin:/home/cleany/cleany`의
`52e44abe89b7103bd832f3cbf806a121c37cb82c`와 미커밋 작업 내용을 읽어 선별했다.
프로파일은 [jetson_20261009](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/)
아래에 보존한다. [provenance.json](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/provenance.json)은
보정 원본, 사용한 내부 보정, 과거 검증 보고서의 경로와 SHA-256을 기록한다.
검증 지표와 실패 또는 미검증 조건은 각 extrinsics YAML과 [캘리브레이션 가이드](CALIBRATION.md)에 있다.
보고서 원본과 이미지 데이터셋은 Jetson의 기록된 위치에 남아 있다.

[mount.png](assets/physical-calibration/mount.png)는 사용자가 제공한 형상 렌더다.
`/Users/hyeon/Downloads/Jetson/mount.png`에서 원본 그대로 복사해 이전 PR 이미지를
교체했다. SHA-256은 `c36c8aff0047bbe3dd34ec6904da879269c4221d2dd68f64d0363cce8d5d9d8b`다.
실물 제어와 calibration 재수행의 증거로 사용하지 않는다.

### 2026-10-21 적용 검증

검증 코드는 `19f3882918f660fe580fe7f7fac0d8869d0a6331`이다.
Jetson Ubuntu 22.04, ROS 2 Humble, Python 3.10.12, MuJoCo 3.11.0의 별도
`/tmp/cleany-calibration-pr-20261021` workspace에서 실행했다.
`cleany_description`의 Git 추적 파일 100개가 해당 commit과 동일한지 hash로 확인했다.
빌드 대상은 description과 인접 MoveIt config 두 패키지이며 전체 workspace 검증은 아니다.

| 검사 | 결과 |
|---|---|
| 루트 `make build`, 별도 `CLEANY_BUILD_ROOT` | 두 패키지 빌드 통과 |
| `colcon test`: description 모델 정합성 | 46개 통과 |
| `colcon test`: 카메라 보정 프로파일 | 10개 통과 |
| MoveIt 설정과 collision 계약 | 18개 통과 |
| 기본 description launch | 명목 헤드 TF, 손목 보정 TF 미발행 확인 |
| `jetson_20261009` description launch | 보정된 헤드와 양손목 TF, RGB alias 5개 확인 |
| MoveIt mock 실행 | Pilz LIN plan-only 결과가 `ABORTED`여서 1개 실패 |

마지막 실패는 수정 전 `main`의 `d5a640a556d473b2a0af59932974b3b3b0aa4732`를
별도 workspace로 빌드해 동일 환경에서 재현했다.
`test_per_arm_position_ik_plan_and_execute`의 `_assert_pilz_lin_is_cartesian`에서
`status == 6`으로 실패하며 기대값은 `STATUS_SUCCEEDED == 4`이다.
이번 변경에서 실행기를 수정하거나 해당 검사를 생략하지 않았다.
Description launch 검사는 localhost 전용 ROS domain 212와 213에서 수행했고
실물 장치 명령은 전송하지 않았다.

원본 로그와 JUnit은 데이터 루트의
`reports/physical-calibration-20261021/`에 보관한다.
Jetson 원본 출력은 `/tmp/cleany-calibration-pr-20261021-output/`,
비교 기준 출력은 `/tmp/cleany-calibration-main-baseline-output/`이다.

### 2026-10-21 시뮬레이터 정합성 재검증

검증 코드는 `e5b26da2e077007003999d1abedb3b67447f6cba`다.
같은 Jetson 환경에서 `make build`로 두 패키지를 다시 빌드하고,
description의 모델 정합성 46개와 보정 프로파일 10개를 통과했다.
검사한 description 추적 파일 100개는 로컬 코드와 SHA-256이 일치한다.
두 URDF 모두 Jetson 모델의 마스트 간격 `0.05`를 유지하며,
`0.08`을 요청하면 오류로 거부하는 회귀 검사를 포함한다.

Gazebo의 `test_cad_frame.py` 6개도 통과했다. SDFormat 12.9.0과 스키마는
공식 ROS APT 패키지를 별도 `/tmp` 경로에 풀어 사용했다.
실제 URDF→SDF 변환 결과에서 RGB와 depth 센서의 위치 및 방향이
변경된 헤드 장착 형상과 TF metadata에 일치하는지 확인했다.
이 검증은 모델 변환 검사이며 Gazebo 동적 실행이나 실물 구동 검증은 아니다.
원본 로그와 JUnit은 데이터 루트의 `reports/pr58-fix-20261021/`에 보관한다.

### 2026-10-21 v8 손목 마운트 반영 검증

검증 코드는 `b6d52578f8e18f9a082cbb43cb2e56f6d8ebd902`다. Jetson의 같은
HEAD와 최신 미커밋 작업에서 v8 마운트만 선별했다. 메시 16개, 형상 JSON,
독립 MJCF와 xacro는 원본 바이트와 일치한다. 실제 보정 YAML 두 개도 이전
수집본과 동일하며, 이번에 hand-eye 보정을 다시 수행하지 않았다.

Ubuntu 22.04 / ROS 2 Humble / Python 3.10.12 / MuJoCo 3.11.0에서 별도
`/tmp/cleany-pr58-wrist-20261021` 사본과 `/tmp/cleany-pr58-wrist-output` 빌드
경로를 사용했다. Description 파일 121개가 로컬 코드와 SHA-256으로 일치한다.
루트 `make build`로 description과 MoveIt config 빌드를 통과했고,
`colcon test`에서 모델 정합성 46개, 보정 프로파일 10개, 마운트 자산 검사 1개,
MoveIt 설정 및 collision 계약 18개가 통과했다. 검사에는 좌우 30개 충돌
primitive의 배치와 크기, m/mm 단위, 실측 기준 치수와 원본 hash를 포함한다.

MoveIt mock 실행 1개는 앞서 `main`에서도 재현한 동일 Pilz LIN 단계에서
`ABORTED`로 실패했다. 이번 검사의 전체 colcon 결과는 이 실패를 포함한다.
새 마운트의 실물 간섭 여유, 카메라 정확도와 로봇 동작은 검증하지 않았다.
원본 로그, JUnit과 파일 manifest는 데이터 루트의
`reports/pr58-wrist-20261021/`에 보관한다.

### PR 58 카메라 수정 후속 검증 (2026-10-22)

기준은 PR 58의 `f8ffd75f741b8da9087de87b01d4ad5ea968ea37`과
`fix/pr58-wrist-camera-pose`의 미커밋 변경이다. MuJoCo의 렌더링 카메라와
optical body/site, sorting 카메라 설정을 v8 마운트 등록 위치·방향에 맞췄다.
Gazebo 레거시 및 팔 포함 CAD 모델도 같은 등록을 사용한다. 손목 마운트의
시야 가림과 좌표계 회귀 검사를 패키지 테스트로 등록했다.

CI 최적화는 PR 61에서 관리하며, 이 변경은 workflow, Makefile과 CI 인프라를
수정하지 않는다. 기존 모델 검사의 xacro 캐시와 Base 검사 등록 변경도 포함하지
않는다. 재촬영은 `tools/validation/capture_cameras.py`를 명시 실행한다.

Ubuntu 22.04 arm64, ROS Humble에서 별도 `builds/pr58-camera-only/` 경로에
관련 6개 패키지를 빌드했다. 최초 선택 빌드는 설치되지 않은 Navigation의
package 환경 파일 때문에 실패했다. Navigation C++ 의존성은 기존과 같이
충족되지 않아 선택 빌드에서 제외하고 촬영에 필요한 Python 소스만 PYTHONPATH로
제공했다. 최초 pytest도 description overlay를 source하지 않아 실패했으며,
새 빌드 overlay를 적용한 최종 description 65개, Gazebo 손목 2개와 도구 9개가
통과했다. 전체 workspace 빌드와 주행 검증을 뜻하지 않는다.

Xvfb/GLX와 software rendering에서 MuJoCo 및 팔 포함 Gazebo CAD의 실제
head/양손목 카메라 촬영을 통과했다. Xvfb는 기존 `pr58-test-cleanup/xvfb-bin`
링크로 추출된 실행 파일을 사용했다. 이 수정은 최신 main `254ffc4`와 함께
임시 worktree에서 충돌 없이 적용됐으며, 원격 CI는 실행하지 않았다.
원본 실패·성공 로그, JUnit, 카메라 PNG와 JSON, 변경 manifest는 데이터 루트의
`reviews/pr58-camera-only/`에 있다. 앞선 전체 backend 검증 원본은
`reviews/pr58-all-backends/`, CI 통합 비교 원본은 `reviews/pr58-pr61-preserved/`에
별도로 보존한다.

| 데이터 루트 아래 경로 | 내용 |
|---|---|
| `fixtures/cad-fortress-command-hold/` | 과거 시나리오 입력 |
| `fixtures/cad-fortress-command-hold-portable/` | local mesh/texture를 고정한 별도 입력 |
| `runs/scenarios/<run-name>/` | 과거 시나리오 결과, journal, 로그 |
| `runs/historical/20261010/artifacts/` | 이전 실행 원본 |
| `reports/legacy-20261010/docs/` | 과거 검증, navigation 분석, ChatGPT 인수인계 사본 |
| `experiments/legacy-20261010/` | 과거 캠페인, SLAM 코드 사본 |
| `experiments/legacy-runtime-cleanup-b3ca816/manifest.json` | 레거시 코드의 commit/hash |
| `migrations/20261010-repository-cleanup/` | 이관 대응표, hash, 검증 로그 |
| `archives/legacy-20261010/scrum-426-scenarios-20261010/worktree.tar.zst` | 이전 scenario worktree |
| `presentation/legacy-20261010/` | 발표 참조, 원본 자료 |
| `builds/<worktree-name>/` | 해당 worktree의 colcon 출력 |
| `builds/ci-optimization-evidence/` | CI 실행 metadata와 원본 로그, 테스트 수집 비교, 종료 경쟁 재현 |
| `reports/documentation-reorganization-20261013/before/` | 이번 문서 개편 전 Markdown 사본 |
| `reports/documentation-reorganization-20261013/manifest.json` | 사본 source commit, dirty 상태, 각 파일 SHA-256 |

Logger 분리와 CLI의 초기 검증은 `CLEANY_DATA_ROOT/reports/evidence/reports/SCRUM-486/validation.json`를 따른다.
`a401f6f`의 Ubuntu 22.04 ARM64/Humble 검증에서는 합성 ROS와 웹 fixture로
Logger 55개, Monitor 80개, Mock 통합 225개(Gazebo 1개 제외), 웹 13개와 CLI 19건을 확인했다.
초기 rosdep 선언과 launch 기본값 누락 오류는 수정 후 재검증했다. 원본은
`ubuntu-vm:/home/ubuntu/Documents/cleany-data/runs/` 아래
`pr62-conflicts-20261023/`와 `pr62-launch-local-review-20261023/`에 보존한다.

`f5e26ab` 기준 Ubuntu 22.04 ARM64/Humble에서 관측 콜백을 300ms 늦춰 Navigation 검사의 기존 실패를 재현했다. 네 속도 토픽을 함께 기다리는 미커밋 수정 후 기본 조건과 지연 조건을 각각 3회 통과했고, 패키지 검사 188개도 오류와 실패 없이 통과했다.
원본과 소스 SHA-256은 `ubuntu-vm:/home/ubuntu/Documents/cleany-data/runs/pr62-navigation-20261024/verification.json`과 같은 경로의 로그에 보존한다.

## 재현, 보존

문서 개편 전 사본의 source commit은 `fbe5854`입니다. Commit, dirty 상태, SHA-256은 보관 manifest, 과거 경로 대응은 `relocation-manifest.json`에서 확인합니다. 원본의 절대 경로와 hash는 보존하고, fixture의 현재 위치와 profile-manifest의 과거 provenance를 구분합니다.

재현할 때는 기록된 commit의 별도 checkout과 해당 환경, 입력을 사용합니다. 다른 브랜치의 자료는 해당 버전의 근거로 취급하며, 제품 결정은 KB 검토, Decision 절차를 따릅니다.

## 인식과 파지 후보 B 분리 검증

A `e780be1`을 기준으로 통합 소스 `94c0fcd`에서 인식과 파지 후보 변경을 분리했다.
Ubuntu 22.04 ARM64, ROS 2 Humble, Python 3.10, pytest 8.4.2 환경에서
`make build-perception-candidates`로 3개 패키지, 기존 소비자 확인을 위한
`make build COLCON_BUILD_ARGS='--packages-up-to cleany_skill_executor --parallel-workers 2'`로
의존성을 포함한 10개 패키지를 별도 경로에 빌드했다. 소비자 빌드는 기존
MoveIt 2.5.9 dependency overlay를 사용했으며 통합 브랜치의 install을 사용하지 않았다.

`make test-perception-candidates`는 493개 통과, 명시 활성화하지 않은 RealSense
실물 검사 1개 skip이다. 기존 수거 소비자 검사 267개도 통과했다. 입력은 합성
RGB-D, 가짜 모델, 저장된 설정이며 실제 Gemini API, TensorRT/AnyGrasp GPU 추론,
카메라, 모터와 전체 수거 시나리오는 실행하지 않았다. launch 두 개의 인자 검사도 통과했다.
초기 분리 검사에서 누락된 의존성, 후속 기능 테스트와 float32 비교를 수정했고,
소비자 노드 생성 검사의 설치 누락은 위 빌드로 해결했다. 최종 검사 실패는 없다.

검증 소스 `changes.patch`, 환경, 실행 명령, 로그와 JUnit 원본은
`/home/cleany/cleany-data/reports/pr-b-perception-20261029T222340Z/`에 보관한다.

### YOLOE 설정 정리 검증 (2026-10-30)

`0830839`에 이 절과 함께 반영한 정리 패치를 적용했다. 이전 34종 설정,
미사용 nano lock과 선택 옵션, 사용되지 않는 IK 예열 설정을 제거했다.
같은 Jetson Humble 환경의 새 빌드 경로에서 `make build-perception-candidates`로
3개 패키지를 빌드하고, 설치된 설정 6개의 hash와 삭제한 설정의 부재를 확인했다.
`make test-perception-candidates`는 492개 통과, RealSense 실물 검사 1개 skip이다.
검사 수가 하나 줄어든 것은 nano 전용 검사를 제거했기 때문이다.
입력은 합성 데이터와 가짜 모델이며 실제 GPU 추론과 실물 장치는 실행하지 않았다.
실패는 없었다. 소스 패치, 환경, 로그, JUnit과 설치 확인 원본은
`/home/cleany/cleany-data/reports/pr-b-yoloe-cleanup-20261029T230830Z/`에 보관한다.

## Mission Mock 조작 통합

| 코드와 환경 | 입력과 결과 범위 | 원본 (`cleany-data/runs/` 아래) |
|---|---|---|
| `171ac80`, Ubuntu 22.04 ARM64, Humble/Fortress | 40초 공유 fixture, study-cafe seat-12, wait_for_next. Mission/Mock/Bridge ROS 208, Monitor 41, 웹 27 검사 통과. Gazebo 139.97초, 두 execution ID와 추가 빈 관측. 깨끗한 Nav2 재빌드는 삭제된 임시 dependency export 경로로 실패했으며 실행은 기존 설치 사용 | `code-review-20261020/verification.json`, `execution-evidence.json`, 로그와 DB |
| 위 코드의 문서 정리 | 링크, 명령과 소스 불변 검사. 최신 #54 병합 전 근거 | `docs-review-20261020/` |
| 이전 40초 fixture | 초기 시간 조정과 ExecuteOne 관측 자료 | `manipulation-timing-20261015-XwwCUn/`, `code-verification-20261015-NS9Nqz/` |

이 수치는 서로 겹치는 검사 묶음이며 합산하지 않는다. 현재 코드의 근거와 과거 실행을 구분한다.

### 최신 PR #54 병합 검증

`171ac80`과 PR #54 `65fdf06`의 병합 작업트리를 Ubuntu 22.04 ARM64, Humble/Fortress에서 검사했다. 원본은 `cleany-data/runs/merge-pr54-20261020/`에 보관한다. 최종 commit과 세부 결과는 해당 폴더의 `verification.json`을 기준으로 확인한다.

| 검사 | 입력과 결과 | 원본 |
|---|---|---|
| Mission과 Mock Action | 두 물체 fixture, ROS domain 193. 208 passed, Gazebo 1개는 별도 실행 | `ros-mock.log` |
| Monitor | ROS domain 194. Cartographer ROS 1개를 제외한 99 passed. 전체 실행의 해당 1개는 `cartographer_ros_msgs` 미설치로 import 실패 | `monitor-ros.log`, `monitor-ros-available.log` |
| 웹 fixture | 33 passed, 실제 Monitor가 필요한 3개는 별도 실행. TypeScript와 production build 통과 | `web-all.log`, `web-build.log` |
| Gazebo와 Mock 조작 | 새 통합 install, study-cafe seat-12, domain 195, wait_for_next. 138.51초에 SUCCESS. 서로 다른 execution ID 2개, 각 17개 세부 동작과 추가 빈 관측 확인. 두 DB 무결성 통과 | `gazebo.log`, `gazebo-evidence.json` |
| 실제 Monitor | FSM, 토픽 선택, 기록 재생, host resource와 Gazebo 지도 검사 통과. 실제 조작 화면과 ExecuteOne 상세도 확인 | `web-live.log`, `web-map.log` |
| 새 통합 build | 생성물 정리 후 16개 package와 Monitor 설치 build 통과 | `build-sim-clean.log`, `build-monitor-clean.log` |
| 계약과 문서 | 최신 #54 FSM, 청소 BT, journal과 보고 model 동일. 로컬 문서 링크 146개 확인 | `mission-core-invariants.json`, `docs-check.json` |

로컬 Nav2 cache의 CMake export가 예전 `/tmp` 추출 경로와 누락된 include 경로를 참조했다. 검증에서는 원래 prefix를 현재 cache에 연결하고 `CPLUS_INCLUDE_PATH`로 설치된 header root를 제공했다. 생성된 export 파일은 수정하지 않았다. 이 환경 보정은 새 VM의 의존성 설치를 대체하지 않으며 재부팅 후 임시 prefix를 다시 준비해야 할 수 있다. 상세 경로와 include 목록은 `nav2-environment.json`, `nav2-build-includes.json`에 남겼다.

### 병합 후 코드 검토

`a7317be` 이후 작업트리를 Ubuntu 22.04 ARM64, Humble/Fortress에서 검토했다. 원본은 `cleany-data/runs/quality-review-pr54-20261020-08o9nj4k/`에 보관하며 최종 소스 hash와 집계는 `verification.json`을 따른다.

| 검토 항목 | 반영 내용 |
|---|---|
| Action remap | Humble의 client와 server에서 이름을 명시적으로 해결하고 내부 CancelGoal 경로도 맞춤. 기본 주소와 사용자 주소에서 두 물체 실행 및 취소 검사 |
| 종료 중 DB 쓰기 | callback과 같은 잠금으로 DB를 닫고 종료 후 tick을 차단. 진행 기록을 ACTIVE로 보존하며 종료를 정지 증거로 만들지 않음 |
| 요청 검증 | 문자열이 아닌 ID와 bool 또는 소수 object ID를 core 수락 전에 거절 |
| 중복과 테스트 | 사용되지 않는 저장소 복구 함수와 미사용 import 제거. 복구 정책은 기존 ExecutionJournal에서 수행. 합성 검증 timeout 대기를 5초에서 2초로 축소 |
| 웹 검사 격리 | UI fixture의 기록 API를 명시하여 실제 로컬 Monitor에 의존하지 않도록 수정 |
| 계약과 문서 | 서버 wall 시각과 steady timeout 구분, 조회와 DB 저장 성공의 차이를 명시. FSM, 청소 BT, 보고 model과 ROS 타입 23종 유지. 웹과 서버의 정상 9단계, 17동작 일치 |

초기 `remap-before.log`와 `ids-before.log`에서 기존 실패를 확인했다. 첫 전체 ROS 실행은 종료 중 SQLite 쓰기와 닫기가 겹쳐 segfault로 중단됐고 `ros.log`에 보존했다. 종료 동기화 수정 후 해당 회귀 검사와 서버 검사 34개가 통과했다. 최종 전체 ROS 검사는 **216 passed, 1 skipped**이며 Gazebo 1개는 별도로 통과했다. 원본은 `ros-final.log`다. 문서 링크 146개와 Python 소스 31개의 syntax, Pyflakes를 확인했다.

Monitor는 99개, 웹 fixture는 33개가 통과했으며 변경한 UI fixture 6개도 다시 통과했다. Cartographer ROS 1개는 이전과 같이 `cartographer_ros_msgs`가 없어 제외했다. Gazebo는 142.52초에 두 execution ID의 수거와 각각 17동작, 추가 빈 관측을 확인했고 Mission DB와 서버 DB의 integrity_check가 모두 통과했다. 마지막 종료 잠금 수정은 별도 동시성 회귀 검사로 검증했다. 원본은 `monitor.log`, `web.log`, `ui-final.log`, `gazebo.log`, `gazebo-evidence.json`, `stage-contract.json`, `static-final.log`에 있다.

## System Supervisor와 Mission Runtime 통합

검증 코드는 `ad472e2`(main `d0afd48` 위 리베이스)다. Ubuntu 22.04.5 aarch64 Distrobox, ROS 2 Humble, Python 3.10.12에서 실행했다. 원본 로그, 지연 JSON과 `verification.json`은 데이터 루트의 `runs/supervision/20261023/`에 있다.

| 검사 | 입력과 결과 | 원본 |
|---|---|---|
| `make test-supervision-core` | 50 passed | `test-supervision-core.log` |
| `make test-supervision` | 69 passed. 실제 Supervisor, safety adapter, base driver와 MockMcu, `mission_runtime`(Nav2 backend, supervised), lifecycle 복구 owner. 센서, Nav2 action, lifecycle manager와 안전 하드웨어 Bool은 합성 | `test-supervision.log` |
| 같은 Supervisor 검사, 18개 CPU busy loop | 47 passed | `test-supervisor-under-cpu-load.log` |
| `make test-mission-core`, `test-base`, `test-navigation` | 77 passed와 17 skipped, 109개, 191개 통과 | 각 로그 |
| `make test-mission-mock` | 233 passed, 2 skipped, 1 failed. `test_process_kill_restart_reports_interruption_without_action_result`는 main `d0afd48`에서도 5회 중 3회 실패 | `test-mission-mock.log`, `flaky-process-kill-*.log` |
| Workspace colcon test, `cleany_manipulation_bt` 제외 | 2218개 중 실패 3. 위 flaky 1개와 이 문서의 기존 MoveIt mock planning `status == 6`(ctest와 xunit 두 번 집계) | `workspace-test-skip-bt.log` |
| `make test` | 이 환경에 `behaviortree_cpp` v4가 없어 `cleany_manipulation_bt` build에서 중단 | `make-test.log` |

Lease 승인과 상태 파일 기록 사이에 Supervisor가 종료되면 lease가 사라지는 결함을 부하 실행에서 발견해 승인 응답 전 기록하도록 수정했다. 수정 전 같은 부하에서 재시작 재확인 검사가 8회 중 4회 실패했고 수정 후 8회 모두 통과했다.

LiDAR 무효 scan 발행부터의 지연이며, 같은 host 한 process 안의 DDS loopback과 Runtime tick 10 Hz 기준이다. 실제 Nav2 controller 정지, 모터 감속과 다중 host 지연은 포함하지 않는다.

| 구간 (5회) | 중앙값 / 최대, 무부하 | 중앙값 / 최대, CPU 부하 |
|---|---|---|
| Lease 철회 | 42 / 46 ms | 24 / 53 ms |
| Nav2 goal cancel 수신 | 46 / 50 ms | 28 / 57 ms |
| Supervisor 정지 확인 (`MISSION_STOPPED`) | 96 / 112 ms | 121 / 201 ms |
| Supervisor 종료 → Nav2 cancel | 0.995 s (heartbeat timeout 1 s) | 0.994 s |

E-stop 회로, MCU 통신 차단 시 모터 정지, 실제 Nav2 lifecycle 복구, 실물 팔 controller, Jetson 다중 프로세스와 SROS2는 검증하지 않았다.

## CI 성능 검증 (2026-10-20)

기준은 Ubuntu 22.04 GitHub runner, ROS Humble의 [실행 39417986298](https://github.com/asm17-moving-agent/cleany/actions/runs/39417986298), commit `11d1ba8`입니다. 전체 job은 804초, workspace 의존성 설치 288초, 빌드와 테스트 279초, 별도 Navigation 빌드와 runtime 123초였습니다. 입력은 해당 commit의 전체 workspace 검사와 기존 motor, Navigation, Gazebo runtime 검사입니다.

| 실행 | Commit | Job 전체 | 결과 |
|---|---|---|---|
| 기존 기준 | `11d1ba8` | 13분 24초 | 통과 |
| [이미지 생성, 게시와 compiler cache 최초 준비](https://github.com/asm17-moving-agent/cleany/actions/runs/39426732345) | `d44b065` | 15분 5초 | 통과 |
| [이미지와 compiler cache 재사용](https://github.com/asm17-moving-agent/cleany/actions/runs/39429236181) | `45df20f` | 4분 37초 | 통과 |

재사용 실행은 기준보다 65.5% 짧았습니다. 이미지 pull 84초, cache 복원 2초, 전체 workspace 빌드와 테스트 90초, motor core 1초, Navigation runtime 30초, Gazebo runtime 46초가 포함됩니다. C++ 입력이 같은 상태에서 compiler cache는 264건 모두 적중했고 전체 빌드는 42.7초였습니다. Workspace 검사 집계는 1,882개, 실패 0개, 기존과 같은 skip 24개이며 CI 도구 9개, Navigation runtime 3개와 Gazebo runtime 1개도 통과했습니다. 베이스 검사를 직접 보고하도록 바꾸어 기존 중첩 래퍼가 집계하지 않던 검사도 XML에 포함됩니다.

5분 이내는 위 cache 적중 실행의 실측값입니다. 의존성 변경이나 주차 갱신으로 이미지를 다시 만들거나, C++ 변경으로 cache miss가 생기면 더 오래 걸릴 수 있습니다. 캐시가 없는 최초 실행도 전체 검사를 통과했습니다.

`1ab6d8a`의 [최초 실행](https://github.com/asm17-moving-agent/cleany/actions/runs/39424741384)은 기존 command guard SIGINT 종료 검사에서 메시지 변환 중 RuntimeError로 실패했습니다. `d44b065`에서 콜백 이후에 종료하도록 수정했습니다. 동일한 Humble 의존성 이미지의 Linux arm64 로컬 재현은 수정 전 10회 중 3회 실패, 수정 후 기존 검사 30회 연속 통과였습니다. 베이스 드라이버의 기능 검사 수집 ID 72개는 변경 전후 동일하며, 기존 모델 검사 함수 24개와 runtime 검사 파일을 유지했습니다.

`d5852af`의 [환경 확인 실패](https://github.com/asm17-moving-agent/cleany/actions/runs/39428868069)는 런타임 `ROS_HOME`이 이미지의 rosdep metadata 위치와 달라 발생했습니다. `45df20f`는 의존성 확인 명령에 이미지 생성 시의 `ROS_HOME`을 지정하며, 재설치 없이 실제 manifest의 의존성 충족 여부를 검사합니다.

원본은 데이터 루트의 `builds/ci-optimization-evidence/`에 보존합니다. `timing-comparison.json`, `baseline-39417986298.json`, `cold-*.json`, `cold-*.log`, `warm-*.json`, `warm-*.log`, `coverage-audit.json`, `base-*-collection.txt`, `shutdown-before.log`, `shutdown-after.log`에서 환경, 실패와 검사 범위를 확인합니다. `cold-artifacts/`, `warm-artifacts/`와 CI artifact는 test XML, apt/pip 버전과 compiler cache 통계를 보존합니다. Simulator와 합성 backend 검사이며 실물 로봇 검증을 뜻하지 않습니다.

## 실물 매핑 패키지 통합

기준은 원격 main `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`, 통합 작업은
`feat/hardware-mapping-integration`이다. 변경 소스 hash는 로컬
`artifacts/mapping-probe-11/source-manifest.json`에 보존한다. 환경은 Ubuntu 22.04
Distrobox, ROS 2 Humble, Linux arm64이며 실물 USB는 열지 않았다.

| 검사 | 입력과 확인한 경계 | 원본 |
|---|---|---|
| 설치 빌드 | lockfile UI와 관련 패키지 20개 빌드 | `artifacts/mapping-probe-11/build.log` |
| 패키지 회귀 | 관련 ROS 검사, ordered permission과 odom reset, 기존 navigation/Logger 회귀 | 같은 디렉터리 `packages.log` |
| 순수 core와 JS | 세션 직렬화, 저장 실패, origin yaw, PID 재사용과 지연 저장 세대 분리, 누름/해제/복구 | `core.log` |
| 설치 launch | `/tmp`에서 실행, synthetic MCU와 합성 room scan, 단일 휠 publisher와 STOP 확인 | `verification.json`, `runtime.log` |
| 세션 저장 | 초기화 2회, 서로 다른 generation과 output, 두 지도/pbstream, Logger flush와 정상 종료 | `runtime/maps/`, `runtime/recordings/` |
| CDR 재생 | domain 175에서 원본 bag 재생, raw/filtered scan, 휠 상태/명령, joint state, odom, TF와 map 수신 | `replay-verification.json`, `replay-verification.log` |
| 종료 소유권 | 기록된 PID와 시작 시각을 대조해 남은 owned child 없음 확인 | `ownership-verification.json` |
| HTTP UI | 설치 자산, token/Origin 거절, 경로 이탈 차단과 CSP | `artifacts/integrated-ui/http-verification.json` |

초기 재현은 `mapping-probe-01`부터 `-10`에 보존했다. 같은 ROS domain을 사용한 검사 간섭,
Logger의 동적 구독 제거 중 InvalidHandle, 종료 래퍼의 신호 전달, symlink 설치 자산의
경로 검사, 수신이 없거나 관측용 주기가 적용된 bag 토픽 누락을 구분해 수정했다.
Logger는 구독별 callback group과 단일 executor로 retired handle이 제어 서비스를 막지 않게 하며,
매핑용 원본은 명시적으로 수신마다 기록한다. ROS 패키지 검사는 domain 174에서 순서대로 실행한다.

브라우저 연결 호스트가 없어 실제 화면 렌더링은 이번 실행에서 확인하지 못했다.
실물 장착 TF, 물리 정지 거리, 바닥 주행과 지도 품질은 이 합성 검사의 범위가 아니다.

의존성 점검에서는 Humble rosdep에 `sllidar_ros2`, `micro_ros_agent` 규칙이 없는 것을 확인했다.
실행 의존성은 manifest에 유지하고, 일반 설치/장치 없는 CI에서는 두 외부 source-build 의존성만
명시적으로 제외한다. CI의 조종 UI 빌드는 별도 Node 24 단계로 추가했다. 이 로컬 검사는 GitHub
Actions 실행이나 실물 드라이버 설치를 대신하지 않는다.

## 매핑과 조종 진단 검증 (2026-11-02)

후속 진단 변경의 commit과 파일 hash는
`artifacts/debug-diagnostics-02/source-manifest.json`에 기록한다. 환경은 로컬 Ubuntu 22.04
ROS 2 Humble container이며 synthetic MCU와 합성 room scan을 사용했다.

20개 패키지 빌드, Python core 61개와 조이스틱 17개, 관련 ROS 검사 428개가 통과했다.
RPC 거절, 동기 및 비동기 예외, 빈 응답, 동일 실패 반복 조회, 시작과 정리의 동시 실패,
진단 파일 저장 실패, 이벤트 큐 포화와 파일 회전은 core 검사로 확인했다.
원본 로그는 같은 artifact 디렉터리의 `build.log`, `core.log`, `packages.log`에서 확인한다.

설치 launch의 두 매핑 세션에서 지도와 pbstream, Logger bag 저장과 STOP 후 종료를 확인했다.
관리자 종료 후 ROS 상태 캐시가 unavailable로 전환됐고 조종 사건 JSON Lines가 남았다.
결과와 세션 경로는 `verification.json`, 종료 후 owned PID 확인은
`ownership-verification.json`에 보존한다. 실물 USB, 실제 주행과 화면 렌더링은 검증 범위에
포함하지 않았다. HTTP 자산 제공과 UI TypeScript 빌드는 확인했다.

## 2026-10-26 ToF와 System Supervisor 통합

기반은 `main`의 `eb2c56a1`과 `feat/system-supervisor`의 `e0d5001e`를 합친
`cbe09bed8cc139e86d85f91201458f4a61693df9`다. 검증 당시 ToF 구현은 이 commit 위의 미커밋 변경이었다.
원본과 설정 SHA256은 `CLEANY_DATA_ROOT/runs/tof-integration/source-manifest.sha256`에
보존한다. KB는 Supervisor 브랜치의 gitlink를 사용하며 내부 내용을 수정하지 않았다.

Ubuntu 22.04 arm64 Distrobox, ROS 2 Humble, Python 3.10에서 `make build-tof`,
`make test-tof-core`, `make test-tof`, `make build-navigation`, `make test-navigation`을
실행했다. 마지막 ToF 패키지 재검사를 반영한 colcon 결과는 207 tests, 오류/실패 0,
선택 시나리오 skip 15다. ToF 자체 34 tests에는 합성 ROS와 pseudo-terminal이 포함된다.
Navigation colcon 결과는 194 tests, 오류/실패/skip 0이다. 중복된 core 검사는 합산하지 않는다.

선택 Mission ROS 검사는 별도로 13 passed, WebSocket 의존성 누락 skip 1이었다.
이후 WebSocket 검사도 단독 통과했다. 최초 추가 실행은 Ubuntu의 websockets 9.1에서
handshake reset으로 실패했으며 Python 3.10 호환 버전인 10.4를 별도 `testdeps` 경로에
설치해 통과했다. 제품 Bridge 코드는 수정하지 않았다. 재현 시 이 경로를 PYTHONPATH에
앞세운다. Gazebo 물리 시나리오는 실행하지 않았다.

원본 결과는 `CLEANY_DATA_ROOT/builds/feat-tof-integration/build/*/pytest.xml`,
`build-navigation/cleany_navigation/test_results/`, `log/`와 `log-navigation/`에 있다.
별도 검사 요약은 `CLEANY_DATA_ROOT/runs/tof-integration/validation.json`에 있다.

공급 펌웨어의 `evidence/range-15hz-2min.jsonl`에서 초기 10Hz 28 frames를 제외한
15Hz 1,817 frames를 오프라인 파싱했다. 평균 유효 zone 33.7347, 기본 보호 coverage를
만족한 frame은 0이었다. 마지막 통계 수신 중 추가 frame도 포함하므로 공급 보고서의
120초 집계 1,816과 범위가 다르다. 원본 경로와 결과는 같은 runs 폴더의
`firmware-replay.json`에 남겼다. 이는 신규 장치 통신이나 실물 정지 검증이 아니다.
실물 commissioning과 한계는 [CALIBRATION](CALIBRATION.md#tof-근접-보호-commissioning)과
[ToF 계약](../ros2_ws/src/cleany_tof/docs/interfaces.md)을 따른다.

## 2026-10-26 ToF 미검출과 방향별 관측 근거

아래 결과는 `905f3d7b673f755a7a7bdbe87b731392558e322e` 위의 작업 트리에서 커밋 전에 검증한 기록이다.
센서는 단독 측정만 완료했고 로봇에 미장착이라는 사용자 정정을 반영했다. 원본 firmware와
실제 센서/모터 설정은 변경하지 않았다. 정책 계약은
[ToF reference](../ros2_ws/src/cleany_tof/docs/interfaces.md)를 따른다.

`range-15hz-2min.jsonl`(SHA256
`ec02366ab656431fed101f7bcc50ca5e9b67b045fc144a1ea4f5f8aaacdcb361`)에서
세션 이전 10Hz 28 frames를 제외한 15Hz 1,817 frames, 116,288 Zone samples를 분류했다.
정상 미검출(count=0/status=255)은 53,109, 품질 조건을 통과한 RANGE는 60,868,
UNRELIABLE은 2,311, NOT_UPDATED는 0 samples였다. 1,817 frames 모두 수신된 관측으로
유지되지만 주행 가능 판정은 아니다. 기존 전체 Zone 조건을 만족한 frame은 0이었다.
이 파일의 firmware-valid 거리 최댓값은 3.22m여서 약 3.5m 미검출의 별도 사용자 관찰을
검출 보장 거리나 자유 공간 하한으로 일반화하지 않았다.

원본의 시작 wrap status, 혼합 품질, 최장 valid 거리 frame 3개를
[test fixture](../ros2_ws/src/cleany_tof/test/data/standalone_frames.json)에 측정 필드 그대로
보존했다. 단독 측정에는 RGB-D의 동시 관측이나 실제 장착 coverage가 없다.
재생 도구는 `tools/validation/tof/replay.py`, 결과는
`CLEANY_DATA_ROOT/runs/tof-coverage/standalone-audit.json`이다. offline 도구의 도착 시각은
검사 재현을 위한 device clock이며 신규 USB 지연 측정으로 해석하지 않는다.

Ubuntu 22.04 arm64 Distrobox, ROS 2 Humble, Python 3.10의 기존 개발환경과 build cache에서
다음 검사를 실행했다. 중복 검사는 합산하지 않는다.

| 명령과 범위 | 결과 |
|---|---|
| `make test-tof-core` | 77 passed |
| `make build-tof`, `make test-tof`와 최종 ToF package 재검사 | 9 packages build; colcon 231 tests, errors/failures 0, 선택 Mission/Gazebo skip 15 |
| ToF package 자체 | 58 tests, errors/failures/skips 0 |
| Humble의 `make test-navigation` | 6 packages build; colcon 199 tests, errors/failures/skips 0 |
| `git diff --check` | 통과 |

기존 finite-depth 기하와 실제 depth coverage 발행 노드→ToF 소비자 연결을 합성 RGB-D,
TF와 odometry로 검사했다. 무효 pixel, stale/잘못된 model/명령 근거의 철회, 정상 미검출의
조건부 속도 제한, 품질 불량 정지, firmware event status=255의 sensor_ready 철회와
기존 lease/Action 취소 경계를 검사했다. 초기 연결 검사에서 test TF가 평가 시각보다
미래였던 fixture를 실제 관측 시각과 일치시켰다. observer 설정이 없는 launch 프로파일도
별도로 검사하여 구성 누락 경로를 수정했다. 검사 조건을 완화해 통과시키지 않았다.

원본 XML과 로그는 `CLEANY_DATA_ROOT/builds/feat-tof-integration/{build,build-navigation,log,log-navigation}`에,
이번 변경의 소스 SHA256과 요약은 `CLEANY_DATA_ROOT/runs/tof-coverage/`에 있다.
합성 근거와 임시 sensor/body geometry는 실물 중첩을 입증하지 않는다. 실제 Nav2 전환을
포함한 full-stack 주행, Gazebo 주행 시나리오, 장착 후 제동과 접근 정확도 검사는 수행하지
않았다. `calibrated=false`를 유지하며 장착 후 항목은
[CALIBRATION](CALIBRATION.md#tof-근접-보호-commissioning)에서 확인한다.

## 2026-10-26 ToF ROI 감지와 책상 접근 단순화

기준 commit은 `162f3788a74c3674d4e1bb1f93df0f3efad8666b`이며 아래는 그 위 작업 트리의
검증이다. 사용자 지시에 따라 정상 NO_RETURN을 장애물 미검출로 처리하고, ToF 전용
RGB-D sweep proof와 전체 Zone coverage 요구를 제거했다. 위 절의 proof 정책은 이전
실행 기록이며 현재 계약은 [ToF reference](../ros2_ws/src/cleany_tof/docs/interfaces.md)다.
기존 head-pan observer, Nav2/Collision Monitor, Mission/Supervisor와 모터 watchdog은
유지했다. 센서와 firmware를 새로 구동하거나 모터 명령을 보내지 않았다.

Ubuntu 22.04 arm64 `ros2-humble` Distrobox, ROS 2 Humble, Python 3.10과 기존 build cache에서
다음 명령을 실행했다. 순수 검사와 package 검사는 일부 중복되므로 합산하지 않는다.

| 명령/범위 | 결과 |
|---|---|
| `make test-tof-core` | 91 passed |
| `make test-tof` | 9 packages build, colcon 247 tests, errors/failures 0, 기존 opt-in Mission skip 15 |
| ToF package 자체 | 74 tests, errors/failures/skips 0 |
| Humble에서 `make test-navigation` | 6 packages build, colcon 198 tests, errors/failures/skips 0 |
| `git diff --check` | 통과 |

navigation의 colcon 198 집계에는 CTest 실행 단위 9개도 포함된다. 개별 pytest/gtest
189 cases와 별도의 독립 검사 수로 합산하지 않는다.

합성 입력으로 15Hz 인접/물리 cluster와 시간 확인, flash/고립/비인접 noise, 중복 frame,
입력 공백 reset, ROI 제외, 단일 비상 return, 거리/해제 hysteresis를 검사했다. ROS 검사에서는
실제 ToF 노드에 합성 frame/odom/lease를 넣어 정상 미검출의 proof 없는 주행 허용,
전부 불량/센서 장애/stale 정지, 일부 불량 Zone의 접근 수락, 평면 소실 abort, 기존
cancel/lease/정지 확인 경계를 검사했다. 드라이버 검사는 pseudo-terminal이며 실제 USB가
아니다. graph의 기존 안전 chain은 identity 대역이다. navigation launch 검사는 ToF on/off와
기존 observer on/off의 네 조합을 조립하며 실제 프로세스를 구동하지 않는다. 제거된 proof
전용 검사와 별개로 원래 finite-depth observer 회귀는 유지했다.

첫 순수 검사에서 ROI 제외 검사와 확인 검사에 같은 timestamp를 재사용한 fixture가
실패했다. 필터의 동일 frame 중복 방지를 유지하고 별도 시나리오 사이 fixture를 reset했다.
재검사 결과는 위 표와 같다. 선택 skip은 성공으로 처리하지 않았다.

기존 `range-15hz-2min.jsonl`을 offline audit로 다시 분류했다. 원본 SHA256은
`ec02366ab656431fed101f7bcc50ca5e9b67b045fc144a1ea4f5f8aaacdcb361`이며 10Hz 28 frames를
제외한 1817 frames에서 RANGE 60868, NO_RETURN 53109, UNRELIABLE 2311 samples였다.
이 재생은 새 USB latency 측정이나 로봇의 주행 허용률 검사가 아니다.

원본 XML은 `CLEANY_DATA_ROOT/builds/feat-tof-integration/{build,build-navigation}`에,
이번 audit와 작업 트리 manifest는 `CLEANY_DATA_ROOT/runs/tof-simplification/`에 보존한다.
센서는 아직 미장착이며 `calibrated=false`다. full-stack Nav2/ToF 또는 Gazebo 주행,
실물 검출/제동/접근 정확도 검사는 하지 않았다. 장착 후 ROI와 거리/시간 후보값 튜닝,
오탐/미탐, 실제 지연과 정지는 [CALIBRATION](CALIBRATION.md#tof-근접-보호-commissioning)의
별도 검증 대상이다. ToF와 다른 센서가 같은 장애물을 모두 미검출할 수 있다.

## 2026-11-02 voxel guard 10Hz 평가 설정

기준은 `162f3788a74c3674d4e1bb1f93df0f3efad8666b` 위의 미커밋 작업 트리다.
기존 ToF 단순화 변경을 보존하고, 여러 감지 경로를 함께 사용하는 구성에서 voxel
충돌 검사와 속도 출력의 기본 주기를 0.05초에서 0.10초로 바꿨다. 공통 navigation,
Gazebo YAML과 node fallback에 같은 값을 적용했다. 지도 갱신, Collision Monitor,
ToF 주기, timeout과 제동 궤적의 `step_s`는 그대로다. 개별 robot profile의 명시적
주기 override는 계속 우선한다.

Ubuntu 22.04 arm64 `ros2-humble` Distrobox와 기존 build cache에서 다음을 확인했다.

| 검사 | 결과 |
|---|---|
| Humble `make test-navigation` | 6 packages build, colcon 198 reported tests, errors/failures/skips 0 |
| Gazebo package와 의존성 build | `colcon build --packages-up-to cleany_gazebo_sim`, 8 packages 성공 |
| 실제 ToF selector/guard의 합성 10Hz 명령 입력 | 정상 구간 49개 출력 sample의 사유 CLEAR, 입력 중단 시 COMMAND_STALE와 zero 확인 |
| `git diff --check` | 통과 |

Gazebo package만 먼저 build한 시도는 해당 install tree의 cleany_base_odometry 누락으로
실패했다. packages-up-to로 의존성을 함께 build해 해결했다. navigation의 198 집계는
CTest 실행 단위 9개를 포함하므로 독립 case 수와 합산하지 않는다.

합성 연결은 별도 DDS domain에서 15Hz frame/odom과 10Hz identity 중계로 실행했다.
상류 명령의 관측 간격 평균은 0.1000초, 최대는 0.1049초였다. 실제 voxel 계산이나
Collision Monitor, Nav2 구동은 이 중계에 포함하지 않았다. 임시 script와 result.json은
`CLEANY_DATA_ROOT/runs/voxel-rate-10hz/`, build와 XML은 기존
`CLEANY_DATA_ROOT/builds/feat-tof-integration/{build-navigation,log-navigation}`에 있다.
실제 CPU 절감, 부하 중 worst-case 지연과 물리 정지 거리는 측정하지 않았다.

## 2026-11-02 ToF PR 검증의 상태 전환 동기화

`162f3788a74c3674d4e1bb1f93df0f3efad8666b` 위의 PR 준비 작업 트리에서 순수 검사
91개를 재실행해 통과했다. 첫 `make test-tof`는 정상 미검출 전환 검사에서 이전 속도
출력과 새 OBSTACLE_HOLD 사유를 비교해 실패했다. release_s 동안의 정상 정지 동작은
유지하고, 테스트가 CLEAR와 정상 출력, AVAILABLE 및 실제 64개 NO_RETURN 수신을 함께
기다리도록 수정했다. 기존 출력 임계값과 센서 상태 조건은 유지했다. 실제 하드웨어는
사용하지 않았다. 재검사 XML은 기존 `build/cleany_tof/pytest.xml`에 있다.

수정 후 make test-tof 재실행은 colcon 247 tests, errors/failures 0, 기존 선택 skip 15였으며
ToF package 74 tests는 errors/failures/skips 0이었다. navigation 198 reported tests와
합성 10Hz 연결 검사는 같은 소스의 앞선 실행 결과다.

## 2026-11-02 PR #71 rosdep 초기화 실패 수정

PR head `ea5adf40dad37c80aa5e4d1da98acbbe942f8769`의 CI run `41363593569`는 테스트 실행 전
의존성 이미지 생성에서 실패했다. cleany_safety_adapter, cleany_system_supervisor,
cleany_tof가 ament_python을 buildtool dependency로 선언해 rosdep이 해석하지 못했다.
세 선언만 제거하고 export의 ament_python build type, 실제 runtime/test 의존성과 CI의
의존성 검사는 유지했다. 기존 로컬 overlay의 package 빌드는 이 rosdep 오류를 검출하지 못했다.

Ubuntu 22.04 arm64 Humble Distrobox와 기존 build cache에서 make test-ci-tools 9건,
make test-tof의 9 packages build와 colcon 247 tests가 errors/failures 0으로 통과했다.
기존 opt-in skip은 15건이며 ToF package 74 tests는 errors/failures/skips 0이다.
공식 기본 rosdep source와 저장소의 ros2_ws/rosdep/cleany.yaml을 별도 ROS_HOME에 준비해
전체 workspace의 rosdep install --simulate --ignore-src --rosdistro humble도 통과했다.
로컬 기본 rosdep source가 초기화되지 않아 발생한 메타데이터 누락은 이 격리 설정으로
해결했으며 전역 설정을 바꾸지 않았다. 실제 CI 이미지는 원격 workflow에서 재검증한다.

로컬 rosdep 로그와 재현 script는 CLEANY_DATA_ROOT/runs/ci-pr71-fix/에, package XML은
기존 builds/feat-tof-integration/build에 있다. 실제 센서나 모터를 사용하지 않았다.

## MoveIt 충돌 메시 최적화와 MuJoCo 동등성 검사 (2026-10-24)

`main`의 `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`에서 충돌 메시 최적화만
분리했다. 형상과 기본값은 Jetson에 적용한 로컬 `cffdddb`, 동등성 검사 수정은
`707538d`를 따른다. STL 17종과 manifest, 선택 옵션, 생성 도구와 검사를 포함한다.
MuJoCo 접촉용 MJCF, 관절, 시각 CAD와 MoveIt 자기 충돌 예외는 main을 유지한다.
cuRobo와 GPU 계산 및 미병합 physical launch의 변경은 포함하지 않는다.

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, Python 3.10.12와 MuJoCo 3.11.0의
별도 main 사본에서 description 패키지를 빌드하고 설치 자산 18개의 hash를
소스와 비교했다. `make test-description-models`에 description의 나머지 검사와
기존 main의 MoveIt 설정, 시뮬레이션 충돌 예외 및 장면 검사를 추가해
122개가 모두 통과했다. 이때 검사한 26개 core 파일은 분리 변경과 동일하다.
NumPy/SciPy 호환 범위와 deprecated API 경고 2개는 남았다.

공통 URDF 기본값 때문에 Gazebo navigation에도 최적화 메시 10개가 전달된다.
이 변경은 Jetson과 같은 기본값을 유지하며, 별도 검토한 Gazebo 원본 형상 고정은
포함하지 않는다. 앞선 122개 검사는 Gazebo 변환 모듈을 실행하지 않는다.
Gazebo 시뮬레이션 실행과 실물 모터 동작을 검증한 결과는 아니다.

원본은 Jetson의 `/home/cleany/cleany-data/reports/collision-main-review-20261024/`에
있다. `main.tar.gz`, `candidate.json`, `installation.json`, `build.log`와
`models-and-main-contracts.log/.xml`에 입력 main, core 파일 hash, 환경과 결과를
보존했다. 해당 검토의 `candidate.patch`에는 별도로 제안한 Gazebo 원본 고정이
포함되어 있으므로 이 PR의 패치와 구분한다. Jetson 운영 적용 기록은
`/home/cleany/cleany-data/reports/collision-deployment-20261024/`에 있다.

### PR 64 최신 메시와 SolidFCL 검증 (2026-10-29)

기준 commit은 `5db7b12a34d2ba85c965d7d783039a947d456afa`이며, 이 절과 함께
반영한 변경을 Jetson의 별도 workspace에서 검사했다. 최신 STL 17종의 삼각형
합계는 4,686개에서 3,966개로 줄었다. 닫힌 볼록 조각 내부의 장애물을 검출하는
`cleany/SolidFCL`을 독립 패키지로 분리하고, 요청한 detector의 로드 실패 시
계획 서비스를 종료하는 검사를 추가했다. 시간 배정, cuRobo와 GPU 실행 변경은
포함하지 않는다.

환경은 Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, MoveIt 2.5.9와 해당 버전의
Pilz 및 MoveGroup overlay다. `reviews/pr64-20261029/`의 분리된 build/install을
사용했으며 운영 install은 참조하지 않았다. 검증한 소스 42개와 설치 STL
17종의 SHA-256이 로컬 PR 작업트리와 일치한다. 소스 목록은
`pr-source-hashes.json`, 설치 확인은 `pr-installation.json`에 있다.

| 검사 | 결과 |
|---|---|
| `make build`에 `COLCON_BUILD_ARGS="--packages-up-to cleany_moveit_config"` 지정 | description, collision, scene mapping, MoveIt config 4개 패키지 빌드 성공 |
| `make test-description-models` | 모델 동등성과 메시 형상 85개 통과 |
| SolidFCL gtest | 내부 장애물, 분리된 틈, ACM, 거리, OctoMap, attached object 및 링크 경계 상자 검사 6개 통과 |
| MoveIt 설정, 시뮬레이션 충돌 예외와 장면 계약 | 18개 통과 |
| 기존 Mock 계획/실행과 종료 검사 | 3개 통과 |
| 설치된 MoveIt 서비스 | 3개 통과: 내부 장애물 36개 검출, 같은 입력의 표면 FCL 누락 재현, 잘못된 detector 요청 시 종료 |

표의 최종 결과는 중복 없는 검사 115개이며 CTest 래퍼는 합산하지 않는다.
서비스 입력은 원본 CAD에서 충돌했던 자세와 장애물 36건의 고정 fixture다.
장애물 없는 상태의 유효성과 접촉 링크도 확인한다. Mock 실행은 가상
ros2_control 하드웨어를 사용했으며 실물 드라이버나 모터 명령을 사용하지 않았다.

처음 세 패키지만 선택한 빌드는 기존 scene mapping의 미설치로 실패해
`--packages-up-to cleany_moveit_config`로 의존 패키지를 함께 빌드했다.
전체 `make test`는 내비게이션 의존성 `dwb_msgs`가 없어 빌드 단계에서 중단됐다.
서비스 검사의 환경변수를 이어 붙인 실행은 `ROS_LOCALHOST_ONLY=0:1`이 되어
2개가 통신 대기로 실패했다. CMake의 `ENV`로 값을 지정한 뒤 해당 3개를
재실행해 통과했다. NumPy/SciPy 경고 2개는 유지된다. 전체 workspace,
Gazebo 동적 실행과 새 성능 비교를 완료한 결과로 해석하지 않는다.

운영 Jetson도 별도로 재확인했다. 로컬 `707538d` 및 Jetson `4f92bc0` 이후의
미커밋 작업에서 대상 소스 32개와 설치 메시 17종이 일치했고, 운영 install의
MoveIt 서비스가 내부 장애물 36개를 모두 검출했다. 양팔 계획은 각 5회, 총
10회 성공했다. 운영 라이브러리를 교체하지 않았으며 trajectory execution을
끄고 수행했다. 이 결과와 PR의 독립 패키지 검증을 구분한다.

원본은 `/home/cleany/cleany-data/reports/pr64-update-20261029/`에 보관한다.
`native/hash-verification.json`, `native/services.json`과 MoveIt 로그가 운영 확인,
`pr-build-*.log`, `pr-models.xml`, `pr-tests/`, `pr-service-final.log`가 PR 검증
근거다. 실패 로그는 `pr-build.log`, `pr-package-tests.log`와
`pr-service-env-failure.log/.xml`에 남겼다. 로컬 사본은
`/Users/hyeon/workspace/cleany/output/pr64-update-20261029/`에 있다.

### PR 64 미사용 경로 정리 검증 (2026-10-30)

`a84daa7`에 이 절과 함께 반영한 정리 패치를 Jetson Ubuntu 22.04 ARM64,
Humble과 MoveIt 2.5.9의 별도 경로에서 검사했다. 미사용 표면 단순화와 전용
검사 3개를 제거했다. 내부 장애물 36건의 자세를 공통 3개로 묶고 원본 입력과
기대 결과가 모두 같음을 확인했다. STL 17종의 hash, 삼각형 3,966개와 조각별
형상 검증 수치는 유지했다. 설치된 메시와 manifest 18개도 소스와 일치한다.

4개 패키지 빌드, 형상 82개, 충돌과 계획 설정 및 Mock 검사 30개가 통과했다.
`make test-moveit-collision`과 기존 Mock 계획, 종료 CTest를 실행했으며 실패와
skip은 없다. NumPy/SciPy 경고 2개는 유지된다. 가상 장애물과 Mock 하드웨어를
사용했고 실물 모터, 전체 workspace 및 Gazebo 동적 검사는 실행하지 않았다.
소스 패치, 환경, 로그, JUnit과 자산 보존 근거는
`/home/cleany/cleany-data/reports/pr64-65-cleanup-20261029T233007Z/pr64/`에 보관한다.

CI 수정 중 main의 c8461d66 병합으로 생긴 Makefile과 TESTING/EVIDENCE의 append 충돌은
양쪽 target과 기록을 모두 보존해 해소했다. 병합 후 CI 도구 9건, ToF 순수 91건과
navigation colcon 198 reported tests가 통과했고 전체 rosdep simulate도 재통과했다.
remote CI는 이 통합 tree에서 다시 실행한다.

## 2026-11-02 책상 접근 ROS 검사의 수신 준비 동기화

최신 main 통합 head d82edca4의 CI run 41369304125는 의존성 이미지 생성과 환경 검증을
통과했지만 전체 2720 reported tests 중 부분 품질 책상 접근 검사 1건에서 goal 거부로
실패했다(32 skipped). 같은 제어 코드는 직전 run 41367607054에서 전체 통과했다.
기존 검사는 publisher sequence 증가만 기다려 서버의 실제 입력 수신과 권한 준비를
보장하지 않았다. 검사에서 서버가 수신한 frame/odom의 나이가 source_timeout_s의
1/4 이내이고 해당 Mission lease가 유효한지 확인한 뒤 요청하도록 바꿨다. 부분 품질
시나리오는 실제 수신 관측에 UNRELIABLE 12개가 있는지도 요구한다. goal 수락, 전진,
cancel과 정지 assertion은 유지했고 운용 코드와 freshness/lease 한계는 변경하지 않았다.

Humble Distrobox의 2 CPU affinity에서 ROS graph 11건과 make test-tof의 colcon
247 reported tests가 통과했다(errors/failures 0, 기존 opt-in skip 15). ToF package 자체는
74건, skip 0이다. 실제 로봇을 사용하지 않았다.

다른 ROS graph 검사 종료 후 두 시나리오를 2 CPU affinity에서 순차 20회씩, 총 40회
반복해 통과했다. 재현 script와 로그는 CLEANY_DATA_ROOT/runs/ci-pr71-fix의
repeat_ready.py와 readiness-repeat.log다.
