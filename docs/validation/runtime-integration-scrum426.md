# SCRUM-426 변경의 runtime-integration 통합 확인

2026-10-10, 노트북 Humble/Fortress. `feat/runtime-integration`에 시나리오 브랜치의
주행·오도메트리·검증 도구와 기존 검증 기록을 병합했다. 통합 후 아래 왕복 2회를
새로 실행했다. 이전 **60/60 runtime SUCCESS, 실제 위치 기준 57/60** 결과는
[원래 연속 검증](scrum-426-scenarios.md)의 별도 실행이며 이번 성공 수에 합산하지 않는다.

- 병합: `5f0906f`, 원본 변경: `0a75789` 오도메트리, `1e3894e` 명령 유지 충돌 검사,
  `a065bcb` 의자 전환·자산 검증·프로필·연속 실행 판정.
- 기존 공유 voxel world, 개발 모니터, headless 회귀 기능을 보존했다. 작업 중이던
  Gazebo launch와 README 3개 파일의 수정은 원래 미커밋 상태로 유지했다.
- 시뮬레이션 튜닝은 `tools/scenario_validation/profiles/cad_fortress_command_hold.yaml`로
  명시해 선택한다. 실물용 기본 보정값으로 승격하지 않았다.
- 통합 검증에서 드러난 command guard 종료 오류는 `4eeb1ac`에서 수정했다.
  SIGINT가 수신 처리와 ROS context 종료를 겹치게 하는 경로를 제거하고,
  Python interrupt로 spin을 끝낸 뒤 정리한다. SIGTERM 처리는 유지한다.

## 검증

- `make build-scenario-runtime`: 통합 소스로 7개 패키지 빌드 성공.
- `make test-navigation`: 실제 testcase **179개** 통과. colcon의 188 집계에는
  CTest 실행 항목 9개가 추가로 들어 있으므로 중복 합산하지 않는다.
- 오도메트리와 시나리오 도구: **67개** 통과.
- Command guard: **10개** 통과. 실제 ROS 명령을 받아 제한 속도로 전달하는 동안
  SIGINT를 보내고 정상 종료를 확인하는 회귀 테스트를 포함한다.
- Gazebo `integration-20261010-command-hold-smoke-r3`: 한 인스턴스에서
  `44 → 16` 왕복 **2/2 SUCCESS**. 각 좌석의 해당 의자만 제거하고 전환 장면을 확인했다.

| 좌석 | 목적지 GT 위치 오차 | 복귀 GT 위치 오차 | 실측 RTF |
| --- | --- | --- | --- |
| 44 | 3.07 cm | 3.76 cm | 1.818 |
| 16 | 2.07 cm | 5.81 cm | 1.847 |

목적지·복귀 모두 8 cm/0.15 rad 추가 대조를 통과했다. 최종 명령과 GT 속도는
정지 상태이며 journal의 두 보고서가 runtime 결과와 일치했다. launch 3개와
자식 30개가 정상 종료했고 남은 실행 프로세스와 traceback이 없었다.
소스 344개와 장면 자산 40개 hash가 유지됐고 메시 로딩 오류도 없었다.

실행 중 parameter readback은 `yaw_scale=1.047`, `PathProgress.scale=20`,
`command_hold_time=0.2`, `command_hold_step=0.01`, `shared_voxel_costmap=true`를 확인했다.
navigation은 Gazebo, 책상 작업은 mock이다. 통합 브랜치에서 60회 연속 시험을 다시
실행한 결과나 물리 로봇 증거가 아니며 실측 RTF 2 이상도 달성하지 않았다.

첫 시도는 시작 좌석 인자 누락으로 시뮬레이션 전에 종료됐다. r2는 왕복 2회에
성공했지만 종료 시 command guard가 `take_message` 변환 RuntimeError로 exit 1이어서
종료 검증 통과로 인정하지 않았다. r3는 위 수정 후 별도 실행이다. 원본 로그는 모두 보존했다.

## 로컬 근거와 실행 환경

요약 감사 결과는 `artifacts/integration-20261010-scrum426/integration-audit.json`,
원본은 `artifacts/scenarios/integration-20261010-command-hold-smoke-r3/worker-00/`에 있다.
빌드·테스트 로그, 실행 전 소스 hash, parameter readback, 기존 작업 백업도 요약과
같은 디렉터리에 보존했다. 원래 60회 캠페인의 대용량 원본은
`../scrum-426-scenarios/artifacts/scenarios/`에 그대로 두었다.

새 overlay는 `ros2_ws/install-scenario-integration-20261010/setup.bash`다.
관련 Make 변수의 build/install/log base를 각각 `build-scenario-integration-20261010`,
`install-scenario-integration-20261010`, `log-scenario-integration-20261010`으로 지정했다.
기존 build/install은 덮어쓰지 않았다. 새 fixture는 원래 시나리오 fixture에 위
command-hold profile을 `prepare_profile.py`로 적용했다.

```bash
python3 tools/scenario_validation/parallel.py \
  --root "$PWD" --container ros2-humble \
  --runtime-overlay ros2_ws/install-scenario-integration-20261010/setup.bash \
  --fixture artifacts/integration-20261010-scrum426/fixture \
  --name NEW_UNIQUE_RUN --count 1 --seat seat-44 \
  --sequence seat-44,seat-16 --repeats 1 --stop-on-failure --domain-base 208
```
