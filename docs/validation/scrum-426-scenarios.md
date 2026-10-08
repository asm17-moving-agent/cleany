# SCRUM-426 시나리오 반복 검증 — 진행 중

2026-10-09, `test/scrum-426-scenarios`, 구현 기준 `076b893`, 초기 검증 도구 `0f40fa4`, 회전 수정 `3d6604d`, 재검증 driver `b9f90d6`.
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

### 노트북과 데스크탑 동시 실행

사용자 요청으로 로컬 `ros2-humble`에서도 별도 runtime install을 빌드했다.
미설치였던 ros-humble-ros-gz-sim을 설치했고 7개 관련 패키지 빌드와 도구 32 tests가
통과했다. domain 120/121, 독립 Gazebo partition/DB/log로 데스크탑과 동시에 실행한다.
로컬 Ogre2 로그의 GL_VENDOR는 freedreno, GL_RENDERER는 Adreno (TM) X2-90이다.
NVIDIA EGL vendor 강제를 제거하고 현재 Mesa 설정을 유지했다. NVIDIA utilization은
이 호스트에서 측정하지 않으며 실제 host/architecture/install prefix를 결과에 남긴다.

`laptop-v2-reference-1343-r1`: 2개 동시 미션 13번/43번 왕복 모두 SUCCESS,
실측 RTF 1.851/1.733, 최종 cmd/GT 정지 및 launch 종료 판정 모두 통과했다.
이 값은 개별 미션 구간 평균이며 한 worker가 먼저 끝난 뒤의 구간도 포함한다.
`laptop-v2-seat01-r1`은 첫 경로 생성과 실제 이동 후 재계획에 실패했다
(NAVIGATION_FAIL, RTF 1.919). 출발 위치의 최초 경로 거부 수정과 전체 왕복 성공을
구분한다. 이 실패의 바뀐 start pose/costmap도 별도로 보존했다.

데스크탑 baseline 전체 82개 좌석의 terminal coverage가 확인됐다:
27개 SUCCESS, 55개 실패. 수정 코드 재검증과 전체 좌석 성공으로 해석하지 않는다.
두 호스트에서 7개 비정상 조건 smoke가 종료됐으며 각각 기존 판정 6/7을 통과했다.
노트북에서 7종 × fresh 5회의 35회 반복을 실행 중이다. 중간 점검에서 13회가
종료됐고 깊이 안전 입력 유실 2회는 모두 미통과였다. 다른 완료 조건은 기대 판정을
통과했다. 이는 전체 반복의 최종 결과가 아니다. 이 유실은 raw depth camera가 아니라
depth clearance adapter 중단이며 물리 센서 분리 시험과 구분한다.
데스크탑의 센서 입력 유실은
STALE_DEPTH와 약 0.40초의 실제 정지를 관찰했지만, cleanup cancel 후 복귀가 실패해
최종 결과가 FAILED였다. 기존 판정에서는 통과하지 않았고 원래 결과를 보존했다.
단순히 기대 결과를 변경해 통과시키지 않고 취소/복귀 정책과 센서 유실 조건을 분석한다.

연속 시험의 의자 전환은 Gazebo scene/info의 실제 top-level 모델 목록과 82개 의자
inventory를 매 미션 전에 비교한다. 정확히 대상 하나만 없을 때 진행하며 원본 응답을
저장한다. 모델 생성/삭제 ACK와 실제 배치 확인을 구분한다. 원래 Gazebo의 실제 응답
(immutable profile의 모델 2개)에서도 protobuf parser를 확인했다. 데스크탑의
`representative-v2-continuous-smoke`에서 44→16 왕복 두 미션이 모두 성공했고,
각 미션의 실제 scene에서 81개 의자와 해당 대상 하나의 부재를 확인했다.
chair_sequence_verified, 최종 정지와 launch 종료 판정이 모두 통과했다.
실측 RTF는 2.018/2.033이다. 관련 도구 테스트는 32 passed.

수정 재검증 종료 뒤, 실패 시도를 보존한 전체 좌석의 마지막 결과로 대표 6개를 골라
좌석별 fresh 10회와 한 인스턴스의 전체 sequence 10회 반복을 실행하는 driver를
준비했다. 좌석 전환 smoke는 먼저 확인하고, 실패하면 연속 반복을 시작하지 않는다.
fresh는 3개 이하 병렬, continuous는 개별 의자 모델을 유지하는 1개이며 성능 profile이
다르다. 데스크탑 재검증이 종료됐으며 기존 실패 55개 중 17/22 두 좌석이 추가 성공했다.
대표는 44/16/17/20/09/35로 선정됐고 fresh 반복이 시작됐다. 대표 반복 driver와
노트북 비정상 조건 반복 queue가 동시에 실행 중이다. 남는 정상 실패의 원인 분석과 최종 반복 결과는 아직 완료되지 않았다.

### 이동 후 재계획 실패의 추가 조사

노트북 1번 실패 지도에서 실제 시작 footprint와 목표 footprint가 안전하고,
시작 근처 3×3 격자 연결도 기존 연속 sweep을 통과했다. 목표/시작 방향을 고정한
XY 탐색은 모두 경로를 찾지 못했다. 추가 고정 방향 7개도 경로를 반환하지 않았다.
room-a4-01의 저장 지도에서도 추가 고정 방향이 경로를 반환하지 않았다.
따라서 격자 시작점 연결이나 고정 방향 추가만으로 해결된다고 판단하지 않는다.

두 저장 지도의 lattice fallback을 별도 offline process에서 재생했다. 기존 2초/100,000
iteration과 진단용 10초/1,000,000 iteration 모두 경로를 반환하지 않았다.
진단용 설정은 실제 runtime에 적용하지 않았다. 재생 map은 plugins 없이 저장 bytes를
불러와 inflation heuristic 가속이 없으므로 실행 시간은 실제 runtime과 직접 비교하지
않는다. 이 결과만으로 연속 공간에서 경로가 불가능하다고 결론 내리지 않는다.
재생 코드/결과/log는 `artifacts/scenarios/planner-replay/full_replay*`,
`full-case*.json`, `full-case*.log`에 보존했다.

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
170 tests, 0 errors/failures/skipped. `make test-scenario-tools`: 로컬/데스크탑 모두 24 passed.

`artifacts/scenarios/planner-replay/`에 원래 1번 snapshot, 임시 C++ replay 코드와
replay_result.json을 저장했다. 수정 라이브러리로 같은 snapshot에서 goal/start heading
두 경로 모두 261 poses, 약 0.019/0.018초에 반환되고 전체 sweep을 통과했다. 이는
저장 비용지도 재검증이며 실제 Gazebo 왕복 성공 증거는 아니다. 기존 조사와 smoke가
종료된 뒤 독립 navigation overlay를 빌드하고 원래 실패 좌석을 재검증한다.

- 월드 보존, 잘못된 terminal mission ID, 실제로 움직이는 최종 상태,
  sensor-loss 취소 이전 정지 판정의 관련 테스트와 순서/소유권/움직임 방향/전체 좌석 coverage 및 선행 smoke 종료 검사: 24 passed (`make test-scenario-tools`).
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

### Body-heading grid 추가와 실제 재검증

`5d5c01b`는 Omni 고정 방향 탐색이 실패한 뒤 지도 해상도 XY 이동과 32개 방향의
제자리 회전을 탐색한다. 기존 cost penalty 5를 유지하며 weighted A*의 기본
heuristic weight 8은 설정 가능하다. 비용 최적성 보장은 없으며 모든 pose/edge와
실제 live map을 전체 연속 footprint로 검사한다. sparse state 저장을 사용하고
기존 공유 2초 예산 중 최대 1/4만 쓰므로 deadline이나 안전 footprint를 완화하지 않았다.
독립 `build-navigation-v3`/`install-navigation-v3`에서 `make test-navigation`:
171 tests, 0 errors/failures/skipped. 좁은 직각 통로에서 방향 전환이 필요한 경로,
정확한 시작/목표, 전체 sweep, cancellation 및 완전한 장애물 barrier를 검증했다.

room-a4-01의 기존 snapshot에서 실제 v3 Omni planner는 539 poses를 0.378초에
반환하고 전체 sweep을 통과했다. 노트북 Gazebo의
`laptop-navigation-v3-room-a4-01-r1`에서는 실제 body-heading grid 경로를 여러 번
반환하고 이동했지만 controller `Failed to make progress`로 NAVIGATION_FAIL로
종료됐다(117.71 wall seconds, RTF 1.666). 정지와 launch 종료는 통과했다.
따라서 경로 탐색 개선과 실제 왕복 성공을 구분하며 해당 좌석이 해결됐다고 주장하지 않는다.
최종 GT는 약 (-5.394,-5.173,0.196rad)이고 guard trace는 PASS/STOP_COMMAND/
COMMAND_TIMEOUT을 기록했다. 이 실패는 controller 경로 추종의 추가 진단 대상이다.

노트북 1번의 이동 후 snapshot은 32방향 탐색에서도 경로가 없었다. 목표에서
역방향으로 탐색해도 76,670 states를 모두 소진해 연결되지 않았다. 이는 해당
이산화의 실패이며 연속 공간의 불가능성 증명은 아니다. 초기 desktop snapshot과
비교하면 lethal/unknown 셀이 1,163개 추가됐고 51개 제거됐다. 실제 가구 관측과
잘못된 검출을 구분하려면 원본 센서/obstacle-memory 근거를 추가 확인해야 한다.
