# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

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

## 2026-10-31 LiDAR 재보정

`6e5a6bd` 기반 미커밋 작업트리를 Ubuntu 22.04 ROS 2 Humble Distrobox에서
검증했다. ESP32-S3 Native USB와 RPLIDAR A1M8을 사용했다.

기존 79.77° 보정으로 수행한 0.5 m 전후 왕복은 전진 ICP가 거부되어
1회 뒤 중단했다. 원본은
`runs/odometry-check-20261031-YBYYj9/RESULT.md`와 같은 경로의 `measurement/`에 있다.

사용자가 지정한 `../cleany_cad/cad/Cleany.FCStd`의 현행 단면과 새 정지 스캔
185개를 정합했다. CAD 해시는
`a8fa5a1717459cf9eb401459f2aabab1880201d82e06aea8e869a66811f6568c`다.
구동 비활성 상태의 WheelState 1,076개, 네 기둥의 단면 정합, 제외 기둥 검증과
기존 주행 스캔 재검증을 보존했다. 표면 잔차 0.88~1.82 mm는 절대 정확도가 아니다.

현재 보정은 yaw 179.878864°, x 141.531308 mm, y 1.145536 mm다.
기존 실물 평가의 전진 규약을 유지하며, 반대 방향인 CAD 표시용 `DriveBase`
등록과 구분한다. 보정 원본과 후속 재측정은
`runs/lidar-cad-recalibration-20261031-3oUYnz/`에 있다.
`calibration_verification.json`은 입력 해시와 저장 스캔 재검증,
`remeasurements/`는 새 실물 주행 원본을 관리한다.

적용 후 전용 검사 11개와 평가 도구 검사 269개가 통과했고,
세 ROS 패키지의 증분 빌드가 완료됐다. 빌드에서 `tests_require` 경고가 있었다.
최종 실물 배치 상태는 해당 `batch_summary.json`을 따른다.

재측정은 전후진 0.20/0.35/0.50 m/s에서 각각 왕복 한 번씩 유효했다.
6개 편도의 odom−LiDAR 위치 차이는 4.83~13.97 mm였다.
첫 횡이동 0.20 m/s에서 ICP 거부가 발생해 전체 배치는 중단됐다.
원본 `batch_20261031_034338_569940/`와 `sway-seed-diagnosis.json`은
측방 방향 불일치의 추가 진단 근거를 포함한다. 종료 후 실제 WheelState에서
구동 비활성, fault 0, 목표 속도와 PWM 0을 재확인했다.

## 2026-10-31 측방 방향 보정

같은 `6e5a6bd` 기반 작업트리와 Humble 환경에서, 사용자가 RL 휠 장착각을
정렬한 후에도 +Y 명령에 실제 오른쪽 이동을 확인했다.
`geometry.lateral_sign: -1`을 명령 역기구학과 wheel odometry에 함께
적용했다. 기본값 +1은 기존 합성 profile을 유지한다.

Core/profile 검사 124개, ROS graph를 포함한 패키지 검사 182개가 통과했다.
실물 실행 중 평가 도구 검사는 공용 실행 잠금 때문에 12개가 진행되지
않았고, 주행 종료 후 재실행한 269개는 모두 통과했다.

저속 횡이동 probe는 양방향 ICP가 유효했고, 이후 세 속도의 전후진과
횡이동 6회 왕복도 유효했다. 편도 위치 차이는 전후 4.86~13.80mm,
좌우 22.61~45.70mm였다. 첫 ±90° 회전의 ICP 거부로 전체 배치는 중단됐다.
±10° 진단에서도 wheel odometry와 LiDAR의 각도 차이가 확인되어
회전 계수는 자동 적용하지 않았다.

원본, 입력 해시, 시험 로그와 결과표는
`runs/odometry-lateral-sign-20261031-m17huy/RESULT.md`에 연결했다.
`full/`과 `rotation_probe/`의 실패도 보존했다. 마지막 WheelState에서
구동 비활성, fault 0, 목표 속도와 PWM 0을 확인했다.

## 2026-10-31 회전 기반 유효폭

사용자가 좌우 휠 간 폭 변경을 확인한 후, 같은 `6e5a6bd` 기반 dirty
작업트리와 Humble 환경에서 회전 기반 유효폭을 보정했다. Wheelbase 350 mm와
반경 63.5 mm를 고정하고 양방향 20°, 45°, 90°의 유효한 LiDAR 편도
21개를 함께 피팅했다.

추정값 136.949 mm를 요청한 5 mm 단위로 반올림해
`base_hardware.yaml`의 `wheel_separation_m: 0.135`로 적용했다.
기구학 유효값이며 실제 장착 폭 측정값은 아니다. 속도별 추정 차이가 남는다.
최종 설정에서 `make test-base`는 182 passed였다.

독립적인 ±90° 확인의 첫 편도는 odom +89.491°, LiDAR +90.979°로
각도 차이 −1.488°였다. 복귀는 inlier 39.47%가 기준 40%에 미달해
중단했으므로 최종 설정의 전체 속도 검증은 완료하지 않았다.
종료 후 실제 WheelState의 구동 비활성, fault 0, 목표 속도와 PWM 0을 확인했다.

수집 조건, 중간 후보, 거부 표본과 해시는
`runs/wheel-width-calibration-20261031-eae72y/RESULT.md`에 연결했다.
`all-angle-fit.json`과 `fit_final_width.py`가 최종 산식을 재현하며,
`confirmation/`은 fitting에 사용하지 않은 확인 원본이다.

## 2026-11-02 오도메트리 변경 분리 검증

`6e5a6bd` 기반 Git tree `fe84c5ae37981eea945851739a64def13d4a3830`의
LiDAR 보정, 측방 부호와 유효폭 변경을 별도 소스와 빌드 경로에서 검사했다.
환경은 Ubuntu 22.04 amd64 ROS 2 Humble Distrobox이며 입력은 합성 데이터와
mock MCU다.

Core/profile 검사 124개, `make test-base`의 pytest 162개와 관련 colcon 검사,
`make test-odometry-evaluation` 269개가 통과했다. 검사 묶음은 중복을 포함한다.
실패는 없었고 빌드에 `tests_require` 경고가 있었다.
소스, patch, tree ID와 실행 로그는 데이터 루트의
`runs/odometry-publish-20261102-YGXFQ3/`에 보존했다.

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
