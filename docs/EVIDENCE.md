# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

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
| `reports/documentation-reorganization-20261013/before/` | 이번 문서 개편 전 Markdown 사본 |
| `reports/documentation-reorganization-20261013/manifest.json` | 사본 source commit, dirty 상태, 각 파일 SHA-256 |

## 재현, 보존

문서 개편 전 사본의 source commit은 `fbe5854`입니다. Commit, dirty 상태, SHA-256은 보관 manifest, 과거 경로 대응은 `relocation-manifest.json`에서 확인합니다. 원본의 절대 경로와 hash는 보존하고, fixture의 현재 위치와 profile-manifest의 과거 provenance를 구분합니다.

재현할 때는 기록된 commit의 별도 checkout과 해당 환경, 입력을 사용합니다. 다른 브랜치의 자료는 해당 버전의 근거로 취급하며, 제품 결정은 KB 검토, Decision 절차를 따릅니다.

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
