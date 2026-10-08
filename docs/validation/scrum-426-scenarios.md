# SCRUM-426 시나리오 반복 검증 — 진행 중

2026-10-09, `test/scrum-426-scenarios`, 구현 기준 `076b893`, 초기 검증 도구 `0f40fa4`, 회전 수정 `3d6604d`, 재검증 driver `b9f90d6`.
로컬과 데스크탑에 별도 워크트리를 만들었다. 기존 runtime-accuracy-review와
feat-navigation-runtime의 소스는 수정하지 않았다. Jira task는 In Review이며
좌석 왕복, 장애물, 경로 실패, 취소와 정지 결과의 재현 가능한 기록을 요구한다.

## 현재 완료 범위 점검 (2026-10-09)

아래는 종료된 queue/status/result/shutdown을 다시 읽은 집계다. 아래의 초기 측정과
진단 기록은 당시 결과이며 현재 진행 상태는 이 표를 우선한다.

| 계획 항목 | 현재 근거 | 판정 |
| --- | --- | --- |
| 독립 worktree/domain/partition/DB/log | local/desktop 별도 root, worker별 metadata | 구현·실행 확인 |
| 데스크탑 병렬 한계 | 4-worker 긴 왕복 3회(12/12 성공), 5-worker 3회(15/15 TIMEOUT), 6-worker 실패 | 완료; 13/43 기준 경로 한정 |
| 처리량 기준 선택 | 2/3 workers 각각 3회 중앙값 63.65/73.25 성공 왕복/시간 | 3-worker 선택 완료 |
| 전체 82개 좌석 | 28 survey batches, 정확한 mission/seat matching과 clean shutdown | 82/82 terminal; baseline 27 성공/55 실패 |
| 실패 좌석 수정 후 재검증 | 실패 55개 + 별도 01 + 13/43 대조, 21 batches/58 missions | 완료; 17/22 추가 성공, latest 29/82 |
| 대표 6개 fresh × 10 | 20 batches/60 missions, 44/16/09/35 각 10 성공, 17/20 각 2 성공/8 실패 | 완료; 44 성공/16 실패 |
| 한 인스턴스 6-seat sequence × 10 | `representative-v2-continuous`, live PID 301654 | 진행 중; 60회 완료 미입증 |
| 7종 이상 상황 smoke | desktop/local 각각 7개 terminal | 각각 6/7 기대 판정 통과 |
| 7종 이상 상황 fresh × 5 | laptop 35 jobs/status/result/shutdown 일치 | 완료; 30/35 통과, sensor-loss 5 미통과 |
| 실패 보존·재생·수정 및 회귀 | rotation/grid/controller 점수 변경, 저장-map 재생, v5 175 tests 및 13/43 왕복 성공 | 부분 개선 확인; 어려운 좌석 왕복은 미해결 |
| 전체 결과 동기화와 최종 종료 감사 | 종료된 JSON/log/bin 증거 local 복사; live continuous는 그대로 유지 | 최종 continuous 종료 후 남음 |

`capacity_report.json`을 다시 생성해 종료된 19개 batch를 반영했다. 초기 configure 실패
`capacity-2-r1`은 terminal report가 없는 중단 사례로 남으며 성공 batch로 집계하지 않는다.
`laptop_scenario_repeat_jobs.json`의 35개 입력과 status를 정확히 대조하고 모든
observer/launch 종료를 확인했다. v5 13/43은 final stopped와 clean shutdown도 통과했다.
구현 관련 test-result를 현재 install의 build 결과에서 읽어 175/0/0/0을 다시 확인했다.
`artifacts/scenarios/plan_completion_audit.json`에 7개 완료 queue의 각 result hash와
verdict를 저장했다. terminal mission/target ID를 직접 대조했고 정상 기대 결과를 제외한
infrastructure/정지/종료 검사 실패는 0개였다. 정상 미션 실패와 예상 fault 결과는 구분한다.

전체 성공률과 raw 정상 실패를 그대로 남긴다. SCRUM-426의 재현 가능한 성공/실패
기록 조건을 전체 좌석 성공 보장으로 바꾸지 않으며, 별도로 합의한 반복 범위도 줄이지 않는다.
Navigation/sensors는 Gazebo 실측이고 perception/planning/manipulation은 mock이다.
안전 입력의 STALE_DEPTH 정지 관찰은 물리 센서/e-stop 검증을 대신하지 않는다.

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

### 노트북 비정상 반복 최종 집계 및 controller 진단

노트북 7종 × fresh 5회가 모두 종료됐다. 입력 35개와 status 35개가 정확히 일치하며
모든 infrastructure exit은 0이다. 각 미션의 실제 terminal/정지/launch 종료를 별도
판정한 `artifacts/scenarios/laptop_repeat_verdicts.json`은 30개 통과, 5개 미통과다.

| 조건 | 반복 | 기대 판정 통과 |
| --- | --- | --- |
| outbound cancel | 5 | 5 |
| return cancel | 5 | 5 |
| synthetic safety stop | 5 | 5 |
| depth-clearance adapter loss | 5 | 0 |
| inserted obstacle | 5 | 5 |
| planner process loss | 5 | 5 |
| blocked target | 5 | 5 |

깊이 안전 입력 유실은 5회 모두 STALE_DEPTH 후 실제 정지(0.404–0.419 wall seconds)를
관찰했다. raw depth camera는 계속 실행되며 물리 D435 유실을 검증한 것은 아니다.
cleanup cancel 후 return 실패로 terminal FAILED가 되어 기존 기대 판정을 통과하지
못했다. 정상 perception/planning/manipulation은 계속 mock이며 물리 로봇 E2E 증거가 아니다.

DWB 진단 구독을 추가한 v3 room-a4-01 재실행은 TIMEOUT이었다(180.38 wall seconds).
`controller-evidence.jsonl`에는 정체 구간에서 zero command가 반복 선택되고
PathOrientation score가 0인 기록이 있다. 같은 frame의 실제 transformed path에서는
현재 pose와 다른 XY/yaw를 가진 첫 grid connector가 관찰됐다. 해당 기록은
원인 후보를 좁히는 근거이며 단일 원인을 확정하는 동기화된 전체 replay가 아니다.
진단 구독의 RTF는 성능 비교에 사용하지 않는다.

`5b7e087`은 body-heading grid의 시작/목표 connector를 회전 후 이동 또는 이동 후
회전으로 명시한다. 두 부분 모두 기존 sweep을 통과해야 하고, 최종 경로의 이동과
회전이 동시에 바뀌지 않는 단위 검증을 추가했다. 독립 v4 overlay에서 171 tests가
통과했다. 실제 v4 첫 Gazebo 실행은 시작점 부근 정체 후 NAVIGATION_FAIL
(67.23 wall seconds, RTF 1.427), 별도 진단 실행은 이동 후 TIMEOUT
(180.36 wall seconds)으로 실패했다. 두 실행 모두 실제 정지와 launch 종료가 통과했다.
connector 표현 개선만으로 왕복이 해결된 것은 아니다. 정체 당시 DWB/local costmap을
함께 수집하려던 진단은 해당 snapshot을 얻지 못했고, 종료된 domain의 read-only
observer는 종료했다. v4의 13/43 동시 왕복 회귀 검증은 두 미션 모두 SUCCESS로
정지 및 launch 종료 판정까지 통과했다(실측 RTF 1.843/1.691). desktop 대표 반복은
아직 진행 중이다.

SCRUM-426의 현재 완료 조건도 다시 읽었다: Gazebo 좌석 이동/대기 위치 복귀 시나리오의
성공 또는 실패를 재현 가능하게 남기는 것이다. issue는 In Review이며 변경하지 않았다.
이 issue 조건과 별도로 이번 계획의 전체 좌석 조사, 대표 fresh/continuous 반복 및
이상 상황 반복 범위를 유지한다. 모든 좌석 성공을 달성한 것으로 해석하지 않는다.

### Reactive mode 구분과 연속 경로 진행 점수

이전 controller 진단에서 pan gate의 이동/회전 분리 규칙을 이 실행의 원인처럼
설명한 것은 맞지 않았다. `runtime_config.configure_safety`는 reactive mode에서
pan node를 observation_only로 설정하고 `/nav2/cmd_vel`을 직접 reactive safety
pipeline에 연결한다. v4의 분리된 connector는 명시적인 경로 표현이며, 이 모드에서
혼합 chassis 명령을 gate가 막는다는 증거가 아니다. 구현 README와 코드 주석도 수정했다.

`laptop-navigation-v4-room-controller-context2.json`에 wheel 정지 구간의 DWB 평가,
actual local costmap GetCostmap 응답, 경로와 TF를 저장했다. map은 평가 이후 수집됐으며
원자적인 동시 snapshot으로 취급하지 않는다. 평가의 523개 후보는 모두 legal이고
520개에 nonzero issued velocity가 있었다. 선택된 정지 후보의 점수는 24.052453995,
PathDist 0, GoalDist 39였다. 전진 후보 여러 개도 전체 7개 critic 점수와 total이
같았다. Nav2 1.1.20 DWB는 total이 엄격히 낮을 때만 best를 교체하므로 이 plateau에서
먼저 나온 정지가 유지될 수 있다. `progress-replay-result.json`은 완전히 평가된
33개 후보의 기하 점수만 재생했고 새 점수에서 전진을 선호했다. 이는 전체 동기화된
controller replay나 실제 도착 증거가 아니다.

`cabd0ea`는 Omni profile에 PathProgressCritic을 추가한다. 남은 local path 길이를
연속적인 metre 단위로 평가해 grid potential이 같은 후보의 경로 전진을 구분한다.
교차점에서 가장 이른 projection을 유지하고 실제 rollout 이동 거리보다 큰 arc
advance를 인정하지 않으며 in-place turn에 병진 전진을 부여하지 않는다. 기존
PathDist/GoalDist/PathOrientation weights와 모든 safety/arrival/deadline guard는
유지했다. 새 critic은 경로 선호 점수이며 안전 허용 조건을 대체하지 않는다.
별도 v5 overlay의 `make test-navigation`: 175 tests, 0 errors/failures/skipped.
plugin registry/config 변경의 package build도 완료했다.

노트북 v5의 room-a4-01/17/20 세 시뮬레이션 병렬 실행은 모두 terminal FAILED였다.
실측 RTF는 각각 1.320/1.325/1.347, wall time은 132.154/127.191/132.129초였다.
room-a4-01과 17은 outbound에서 실패했고, 20은 target navigation OK 및 mock 작업
완료 후 return에서 NAVIGATION_FAIL이었다. 세 실행 모두 final stopped와 launch
종료 판정이 통과했다. 새 진행 점수만으로 왕복을 해결하지 못했다. v5의 13/43
대조 왕복은 별도 2개 병렬 실행에서 모두 SUCCESS였다(실측 RTF 1.883/1.701).
두 미션 모두 final stopped와 launch 종료를 포함한 전체 기대 판정을 통과했다.
기존 desktop v2 반복은 중간에 빌드나 source를 교체하지 않고 계속한다.
별도 저장-map 실험에서는 전체 SAT sweep을 통과한 v3 room 경로의 한 pose가
stock FootprintCollisionChecker에서는 lethal로 판정됐다(index 199). 이것이 현재
upcoming-path 재계획의 원인인지 아직 입증하지 않았고 IsPathValid를 우회하지 않았다.


### 두 호스트 병렬 실행 진행 결과

노트북 35회 이상 상황 반복은 모두 종료되어 30/35 기대 판정을 통과했다.
깊이 안전 입력 유실 5회의 terminal 불일치와 실패 증거를 그대로 유지한다.
데스크탑의 대표 좌석 fresh 반복은 60/60회 수행 완료, SUCCESS 44회/FAILED 16회다.
44/16/09/35는 각각 10/10 SUCCESS, 17/20은 각각 2/10 SUCCESS와 8/10 FAILED였다.
이는 v2 결과이며 노트북의 v5 변경과 섞어 개선 효과로 해석하지 않는다.
한 인스턴스에서 6개 좌석 순서 × 10회 연속 실행은 별도로 진행 중이다.
전체 계획 완료는 아직 선언하지 않는다.

노트북 v5 3-worker 실행의 호스트 전체 CPU delta 평균은 60.9%(최대 71.4%),
MemAvailable 최저 13.55 GiB였다. Ogre2 log의 renderer는 freedreno/Adreno X2-90이다.
NVIDIA monitor가 없으므로 GPU utilization은 None(미측정)이며 0%로 해석하지 않는다.
이 어려운 좌석 3개 실행의 RTF/실패만으로 최대 병렬 수나 CPU 단일 병목을 확정하지 않는다.


### v5 20번 복귀 실패 저장 지도 재생

`worker-02/navigation/planner_failures/failure.json`과 각 720×420 bytes의 search/live
snapshot 크기를 확인한 뒤 원래 geometry replay를 수행했다. 시작과 목표 footprint는
각각 안전했지만 시작점의 목표 방향 회전은 불가했고 fixed-heading 두 경로는 실패했다.
같은 search map의 32방향 SE2 weighted search(weight 8, 기존 cost penalty 5)는
129,456 states를 0.473초에 소진해 연결되지 않았다(timeout=false). 재생 입력은
`laptop-navigation-v5-hard-seats-r1/worker-02/navigation/planner_failures`, 결과는
`planner-replay/v5-seat20-return-geometry.json`와 `v5-seat20-return-se2.json`이다.
이는 시작 footprint 자체의 충돌이나 runtime 2초 예산 소진으로 설명되지 않는
저장 지도/이산 탐색의 실패다. 센서가 실제 가구를 반영했는지, 관측 오차가 연결을
끊었는지와 연속 공간의 실제 통행 가능성은 이 결과만으로 확정하지 않는다.
