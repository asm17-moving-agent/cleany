# SCRUM-426 시나리오 반복 검증 — 진행 중

2026-10-09, `test/scrum-426-scenarios`, 구현 기준 `076b893`, 검증 도구 `0f40fa4`.
로컬과 데스크탑에 별도 워크트리를 만들었다. 기존 runtime-accuracy-review와
feat-navigation-runtime의 소스는 수정하지 않았다. Jira task는 In Review이며
좌석 왕복, 장애물, 경로 실패, 취소와 정지 결과의 재현 가능한 기록을 요구한다.

## 입력과 의자 조건

전체 설정의 82개 좌석과 82개 의자 대응을 검사했다. 기존 facility fixture에서
누락된 43/13번 의자를 원본 study world에서 복원한 다음, 각 대상의 의자 하나만
제거한다. 82개 월드 모두 다른 모델을 유지하고 해당 의자만 제거하는 검사를 통과했다.
실제 실행마다 입력과 생성 월드 hash, 제거한 의자 이름, ROS domain과 Gazebo partition,
DB/결과 경로를 기록한다. 연속 좌석 시험에서는 이전 의자를 복원하고 다음 의자를
제거한다. 이때 의자 모델은 immutable static grouping에서 제외한다.

## 초기 병렬 측정

Ryzen 5 9600X, RTX 5070, ROS 2 Humble/Fortress, Ogre2 EGL headless,
요청 RTF 4, physics step 0.0066667 s, reactive safety 및 obstacle memory.
실측 RTF는 각 미션의 `/clock` 증가량 / wall seconds다. 준비/종료 시간은 제외한다.
아래는 아직 경계 반복과 전체 좌석 검증 전의 첫 측정이다.

| 개수 | 43번 개별 RTF | 13번 개별 RTF | 결과 |
| --- | --- | --- | --- |
| 1 | 2.153 | 2.145 | 두 경로 왕복 성공 |
| 2 | 1.681–1.684 | 1.636–1.689 | 두 경로 왕복 모두 성공 |
| 3 | 1.216–1.227 | 1.192–1.216 | 두 경로 왕복 모두 성공 |
| 4 | 0.804–0.822 | 0.784–0.813 | 두 경로 왕복 모두 성공 |
| 5 | 0.503–0.542 | 0.484–0.495 | 43 성공, 13 모두 180초 TIMEOUT |

4개/5개 측정 전체 구간의 CPU 평균 사용률은 약 89–92% / 96–98%였다.
GPU 최대 사용률은 16% 이내이고 5개에서도 MemAvailable은 약 31 GiB 남았다.
현재 구성의 초기 포화는 CPU와 함께 나타났다. 독립 CPU/메모리 대역폭 profiler로
세부 원인을 분리한 측정은 아니다. 단일 미션 평균 RTF 합은 참고값이며 실제 batch
처리량과 동일하다고 취급하지 않는다.

2개 첫 시도에서는 controller configure 응답 누락 뒤 준비가 끝나지 않았다.
해당 실패와 중단된 다른 인스턴스의 raw 로그를 보존했다. 검증 도구를 autostart=false로
바꾸고 discovery 후 localization/navigation/safety manager STARTUP을 순서대로
요청한 뒤 2–5개 실행이 준비됐다. 런타임 안전/도착/deadline 조건은 변경하지 않았다.

## 검증 상태와 남은 작업

### 전체 좌석 조사 중 확인한 회전 경로 거부

원래 코드로 조사 중인 첫 21개 결과는 SUCCESS 9개, FAILED 12개다. 첫 6개는
출발점에서 이동 없이 약 10.7초에 NAVIGATION_FAIL로 종료됐다. 1번의 planner
failure.json/search.bin/live.bin에서 시작과 목표 footprint 자체는 충돌하지 않았다.
180도 회전을 하나의 conservative sweep으로 검사하면 arc padding이 약 0.67m까지
커져 실제 회전 궤적 밖의 벽/책상 셀도 포함한다. 각도 구간별 연속 sweep으로 같은
회전을 검사하면 충돌하지 않는다. 원래 실패 로그는 보존했다.

planner endpoint 회전과 live path 검증을 각 모서리의 각도 이동이 지도 해상도의
절반 이하인 구간으로 나눴다. 각 구간은 기존 full-cell SAT/arc bound를 유지하며
실제 회전 중 접촉, unknown, padding과 cancellation은 계속 거부한다. Controller
rollout 검사는 바꾸지 않았다. 로컬 ROS Humble container에서 `make test-navigation`:
170 tests, 0 errors/failures/skipped. `make test-scenario-tools`: 21 passed.

`artifacts/scenarios/planner-replay/`에 원래 1번 snapshot, 임시 C++ replay 코드와
replay_result.json을 저장했다. 수정 라이브러리로 같은 snapshot에서 goal/start heading
두 경로 모두 261 poses, 약 0.019/0.018초에 반환되고 전체 sweep을 통과했다. 이는
저장 비용지도 재검증이며 실제 Gazebo 왕복 성공 증거는 아니다. 기존 조사와 smoke가
종료된 뒤 독립 navigation overlay를 빌드하고 원래 실패 좌석을 재검증한다.

- 월드 보존, 잘못된 terminal mission ID, 실제로 움직이는 최종 상태,
  sensor-loss 취소 이전 정지 판정의 관련 테스트와 순서/소유권/움직임 방향/전체 좌석 coverage 검사: 18 passed (`make test-scenario-tools`).
- 82개 좌석별 의자 대응과 제거 검사: 통과.
- 4개 성공/5개 실패 경계의 3회 반복 및 6개 대조: 완료.
  4개에서는 총 12개 긴 왕복 SUCCESS, 5개에서는 총 15개 TIMEOUT,
  6개 대조도 전원 실패이고 개별 RTF는 0.335–0.343이었다.
- 4/5개 경계는 세 번 모두 같은 성공/실패 결과를 보였다. 이 결론은 두 기준 경로와
  현재 profile/deadline에 한정하며 전체 좌석의 성공 한도를 뜻하지 않는다.
- 긴 기준 경로의 첫 batch 처리량: 1개 38.7, 2개 63.6, 3개 65.2, 4개 60.9 성공 왕복/시간.
  2/3개 차이가 작아 각각 3회 측정의 중앙값으로 선택한다.
- 실제 capacity queue process 종료와 terminal status 확인 후 처리량 반복 및 전체
  82개 좌석을 수행하는 survey driver가 실행 중이다.
- 전체 82개 좌석 실제 왕복, 대표 좌석 반복, 연속 좌석 전환: 미완료.
- 장애물, 차단, planner 유실, 이동/복귀 취소, safety fault, sensor 유실:
  실행 도구 구현, 전체 좌석 terminal coverage 확인 후 smoke를 시작하는 driver 실행 중.
  실제 smoke 및 반복 결과는 아직 미완료.

정상 실패를 성공으로 바꾸지 않는다. sensor, freshness, 속도, 8 cm runtime 도착
게이트 및 180초 navigation timeout은 원래 설정을 사용한다. 동적 장애물의
clearance는 configured padded XY body와 sampled GT 사이의 거리이며 실제 contact
sensor나 팔의 3D 충돌 측정으로 표시하지 않는다. E_STOP 알림은 synthetic stimulus이며
물리 e-stop producer의 증거가 아니다. Navigation은 Gazebo 실측, 책상 작업은 mock이다.

Raw 근거: 데스크탑 `artifacts/scenarios/capacity-*`의 trace/result/shutdown,
resources.json, fixture/seat_inventory.json. 실행 방법과 자동 판정은
[검증 도구 README](../../tools/scenario_validation/README.md)를 따른다.
