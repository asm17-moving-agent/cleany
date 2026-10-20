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
| `builds/ci-optimization-evidence/` | CI 실행 metadata와 원본 로그, 테스트 수집 비교, 종료 경쟁 재현 |
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

## CI 성능 검증 (2026-10-20)

기준은 Ubuntu 22.04 GitHub runner, ROS Humble의 [실행 39417986298](https://github.com/asm17-moving-agent/cleany/actions/runs/39417986298), commit `11d1ba8`입니다. 전체 job은 804초, workspace 의존성 설치 288초, 빌드와 테스트 279초, 별도 Navigation 빌드와 runtime 123초였습니다. 입력은 해당 commit의 전체 workspace 검사와 기존 motor, Navigation, Gazebo runtime 검사입니다.

`1ab6d8a`의 [최초 실행](https://github.com/asm17-moving-agent/cleany/actions/runs/39424741384)은 기존 command guard SIGINT 종료 검사에서 메시지 변환 중 RuntimeError로 실패했습니다. `d44b065`에서 콜백 이후에 종료하도록 수정했습니다. 동일한 Humble 의존성 이미지의 Linux arm64 로컬 재현은 수정 전 10회 중 3회 실패, 수정 후 기존 검사 30회 연속 통과였습니다. 베이스 드라이버의 기능 검사 수집 ID 72개는 변경 전후 동일하며, 기존 모델 검사 함수 24개와 runtime 검사 파일을 유지했습니다.

원본은 데이터 루트의 `builds/ci-optimization-evidence/`에 보존합니다. `baseline-39417986298.json`, `cold-39424741384.log`, `coverage-audit.json`, `base-*-collection.txt`, `shutdown-before.log`, `shutdown-after.log`에서 환경, 실패와 검사 범위를 확인합니다. CI artifact는 test XML, apt/pip 버전과 compiler cache 통계를 보존합니다. Simulator와 합성 backend 검사이며 실물 로봇 검증을 뜻하지 않습니다.
