# 자율주행 통합 검토 — 2026-10-04

## 주행 연속성 개선 비교 (최신)

동일 GUI 조건에서 개선 전후 각각 두 번 43→home→13→home을 수행했다. 왕복 평균 시간은 43 36.5→30.0초, 13 157.2→121.4초다. 13번 복귀 실제 정지는 29.0→12.9초, 0.2초 이상 정지 구간은 평균 45.0→13.5회였다. 방향 유지 후보와 soft switching cost를 추가했으며 각 후보는 기존 충돌·가속도 검사를 거친다. 검증한 GoalDirectedController/radius .75를 omni 기본값에 반영했다. 아래의 과거 DWB/radius0 기본값 설명은 이 결과 이전의 기록이다.

관측 범위 판정은 구현했으나 전체 body sweep 관측에 실패해 **shadow 전용**이다. 기존 각도 가드와 안전/시간 기준은 유지한다. 실측 2배속, 근접 사각지대, 팬 기계적 경계의 긴 재정렬, 복귀 위치 정확도는 해결되지 않았다. 43번 복귀 GT 오차는 평균 7.5→8.5cm로 악화됐으며 모든 구간의 개선을 주장하지 않는다. Navigation 142 tests 통과 및 모든 GUI/런타임/backend 종료를 확인했다.

[전체 비교표·개별 실행·한계](../artifacts/navigation-continuity-20261004/RESULTS.md), [실제 속도 비교](../artifacts/navigation-continuity-20261004/return-speed.png), [안전 설정·종료 불변 검사](../artifacts/navigation-continuity-20261004/validation.json).

## GUI 재시험 추가 결과

Ogre2 GUI를 실제로 열어 홈→43→홈→13→홈을 새 DB로 재시험했다. 두 미션 모두 SUCCESS이며 43 **32.64초/실측 1.372배**, 13 **140.66초/1.378배**였다. GT 도착 위치 오차는 **5.35/0.79cm**, 홈 복귀 오차는 **7.17/11.44cm**다. 안전 기준과 360초 wall 제한은 유지했다. **실측 2배속과 홈 위치 정확도 문제는 남는다.** 화면에서 주행을 확인하고, 종료 후 GUI/런타임/backend 잔여 없음과 최종 속도 0을 확인했다.

[GUI 시험 상세·화면 증거](../artifacts/navigation-reanalysis-20261004/gui-runtime/RESULTS.md). 아래는 직전 headless 비교 결과다.

## 2026-10-04 재분석 결과

**실측 2배속 목표는 미달이다.** 새 출발 조건에서 홈→43→홈→13→홈은 전후 모두 SUCCESS였으며, 총 wall 시간은 43 **178.48→32.42초**, 13 **501.01→130.03초**다. 미션 실측 RTF는 각각 **0.397→1.532**, **0.394→1.477**이다. 후 실행은 Ogre2/6.67ms/요청 RTF4이며, 설정값을 실측 2배속으로 보고하지 않는다.

장애물 메모리의 TF blocking wait를 제거하고 격자/PointCloud2 변환을 typed array로 바꿨다. 구독자 없는 RGB는 lazy bridge로 바꾸되 depth/CameraInfo/LiDAR는 계속 구독한다. safety/freshness/속도·가속도/도착 기준/360초 wall 제한은 보존했다. Ogre1의 빠른 결과는 LiDAR 이상으로 제외했고, GPU 임시 설정은 복원했다.

GT 도착 오차는 후 43 **4.80cm/0.03°**, 13 **0.81cm/3.12°**다. 홈 복귀 오차는 **7.30cm/2.27°**, **13.09cm/1.73°**이므로 위치 정확도 개선도 주장하지 않는다. 최종 cmd_vel/GT twist 0, navigation 133 및 simulator 관련 32 tests 통과. 실측 2배속, 반복 신뢰성과 홈 위치 정확도는 남은 과제다. navigation은 Gazebo, 청소 등은 mock이다.

[전체 전후 비교·원인·제외 실험](../artifacts/navigation-reanalysis-20261004/RESULTS.md), [전 결과](../artifacts/navigation-reanalysis-20261004/before/summary.json), [후 결과](../artifacts/navigation-reanalysis-20261004/after-ogre2/summary.json), [최종 종료 감사](../artifacts/navigation-reanalysis-20261004/final-shutdown.json).

## 아래는 재분석 전 기록

검증은 **미완료**다. 최신 실제 시험 `comparison-20261004-172000`은 O2 빌드와 opt-in GoalDirectedController(radius .75)를 사용했다. 43번 도착·복귀 SUCCESS181.95초, GT 도착 오차5.88cm/4.12°. 13번은231.86초 outbound 뒤 도착(GT2.41cm/2.79°)했으나 복귀360초 제한을 넘겨 전체 FAILED/TIMEOUT595.34초다. 종료 시 홈까지GT2.06m 남았다. 실제 navigation은Gazebo, 청소 실행은mock이다.

개선 전 기록은43 GT12.84cm/4.57°,13 GT10.19cm/6.41°로 이번 도착 오차는 감소했다. 그러나 개선 전13은복귀도성공했으므로 전체신뢰성향상을주장할수없다. 43의이전출발은이미책상가까웠으며GUI·시작자세·재부팅조건차이때문에단일실행의원시시간은통제된A/B가아니다. 직전161523보다43총시간327.74→181.95초,13도착시간초과→도착성공이지만O2만의효과는분리되지않았다.

13 복귀의 신선한 표본을 적분하면 TURNING_PAN89.37초, WAIT_FRESH_DEPTH45.78초, PASS225.04초다. PASS중 평균 요청 속력은 outbound .1128m/s 대비 return .0705m/s다. 3Dguard는353.76초 PASS_OBSERVED_SPACE/7.10초 STOP_UNAVAILABLE이며 동일 guard state의 requested/output 쌍에서 요청속력>.01m/s를10%이상감속한 표본은없었다. 즉 카메라대기뿐아니라DWB의낮은요청속력도조사해야한다. 전체체인토픽은원자적으로동기화되지않았고critic scores가없어낮은속력선택원인은확정하지않는다.

복귀30초구간평균 owned CPU398.2%(100%=1논리코어),RTF .4005다. 직전161523후반의869%수준폭증은이번에같은수준으로재현되지않았다. 낮은RTF와출력·관측대기가360초wall deadline을소모하지만timeout이나안전조건을느슨하게하지않았다.

후속 평가 수집기는 `CLEANY_CAPTURE_DWB=return`으로 실제 활성13미션의 RETURNING 단계에만 제한적으로 구독한다. backend read가실패하거나1초이상오래되면구독 admission을닫는다. 최대10개평가,20초간격,최대3초구독창으로outbound에서한도가소진되는문제를막는다. 기록에는활성미션context와latest-received표본한계를포함한다. 이진단수정은py_compile통과했고실제복귀중작동은아직검증하지않았다.

빌드는기본RelWithDebInfo이며navigation132tests가통과했다. 같은부팅의O0/O2 fixed-heading회귀중앙값116→18ms이나collision회귀전체시간에서는개선을입증하지못했다. Planner double-free는단일lattice link와정상종료회귀/별도planner_server exit0으로검증했다. 다른노드종료오류는별도미해결대상이다. 기본근거리정책은DWB/radius0으로시험기능비활성을유지한다.

시뮬레이션/Nav2/mission/backend/observer 종료감사remaining=[]/untracked=[]를확인했다. 원본증거는172000의RESULTS.md,comparison.json,phase-command-analysis.json,return-summary.json,shutdown.json이다. 아래기록의'최신'은각과거단계당시의결과다.

## 이전 진단 경과 (124528–133502)

검증은 **미완료**다. 마지막 43 → 13 순차 시험 `comparison-20261004-124528`에서 43번은 책상 앞 도착과 홈 복귀까지 SUCCESS(334.26초, GT 도착 오차 6.81cm / 0.58°)였지만, 13번은 TIMEOUT으로 목표 1.63m 앞에서 종료됐다. Depth 자기 필터 최적화 후 해당 노드 CPU 평균은 약 53~54%에서 36~38%로 낮게 관측됐으나, 주행 시간과 13번 성공은 개선되지 않았다. 실행 조건이 달라 통제된 인과적 A/B 비교는 아니다.

최신 `comparison-20261004-130250`은 홈에서 출발한 13번 **단독 진단**이다. 360.61초에 TIMEOUT으로 목표 1.45m 앞에서 종료됐다. 제한적으로 수집한 DWB evaluation 8개 중 한 주기에서 523개 후보 모두의 첫 출력 속도는 0이었지만, 선택 궤적은 이후 9.8cm 이동과 0.231rad 회전을 예측했다. 이를 정지 목표가 최저 점수로 선택됐다고 읽으면 안 된다. Generator의 실제 시간 간격과 reset 원인을 같은 주기에서 직접 관측해야 한다. 일부 후보의 점수 목록은 short-circuit으로 끝나므로 완전한 평가로 간주하지 않는다.

이번 진단에서는 후보 점수, 가속 제한, 안전 가드, 센서 기준을 변경하지 않았다. Runtime 준비 검사와 실제 엔코더 125ms 설정을 확인한 뒤 실행했고, 종료 후 정확한 observer argv와 cwd까지 감사한 shutdown 결과는 remaining=[]다. 상세 결과와 원본 증거는 최신 실행 디렉터리의 `RESULTS.md`, `selection-analysis.json`, `dwb-selection-*.json`에 보존한다. 아래는 단계별 역사적 증거이며 최신 성공 주장으로 읽지 않는다.


최신 시간 진단 `comparison-20261004-131909`의 generator 6,836개 주기(dropped=0)에서 첫 시간 구간 0은 62회(0.9%), 모든 첫 출력 0은 11회(0.16%)였다. 같은 ROS 시각 59회와 reset 3회를 구분했고 역행은 없었다. 이 현상을 장시간 지연의 주원인으로 채택하지 않는다. 정상 시간 간격에서도 측정 속도 0은 1,631회였다. 평균 엔코더 피드백·카메라 대기·작은 재출발 명령의 연동이 다음 개선 대상이다. 진단 on/off 궤적 동일 회귀를 포함한 generator 21개 테스트와 build가 통과했다. 이번 13번 독립 실행도 TIMEOUT이며, 종료 후 exact observer 감사 shutdown remaining=[]다.


최신 비용 기준 유지 실험 `comparison-20261004-133502`은 13번 홈 출발 단독 시험이며 361.24초 TIMEOUT, 목표 1.97m 앞이었다. 저장된 6개 후보 주기에 대한 scale 4/8/16 재점수는 선택을 바꾸지 않았다. 이어 기준 방향 0.5초 유지 옵션을 시험했지만 진행 개선을 입증하지 못해 소스와 옵션을 제거하고 기존 CommandBearing으로 복구했다. 실험 스냅샷은 보존했다. 복구 후 generator 21개 테스트와 navigation build가 통과했다. 전 실행의 lifecycle 초기화 실패는 준비 검사에서 미션 제출을 막았으며 성능 실패로 집계하지 않는다. 두 실행 모두 종료했고 shutdown remaining=[]다. 계획 body yaw 회전이 홀로노믹 경로에서도 필요한지는 같은 지도와 full-body sweep으로 다음에 확인해야 한다.


후속 구현은 holonomic lattice 우회의 XY 위치를 유지하면서 몸체 yaw를 시작 또는 목표 방향으로 고정하는 대안이다. 기존 convex swept/full-cell collision 검사를 private search map과 최신 live map에 적용하고 불가능하면 기존 경로를 유지한다. 좁은 통로에서 회전이 필요한 경우에는 대안을 거절한다. Planner 7개와 navigation 패키지 전체 117개 테스트, build가 통과했다. 실제 시설지도 선택과 43/13 왕복 개선은 아직 검증 전이며 최신 실제 성공으로 주장하지 않는다.

## 범위와 증거 경계

통합 워크트리 `feat/runtime-integration`에서 Gateway → control bridge → Mission Runtime → Nav2 → pan gate → 2D collision monitor → 3D voxel guard → Gazebo base 전체 흐름을 검토했다. Navigation은 실제 Gazebo/ROS 실행이며 perception/planning/execution의 청소 모듈은 mock이다. 실제 장비의 청소·조작 접근과 안전을 입증하는 시험이 아니다.

실행 자료는 `artifacts/gui-integration-20261003/`에 보존한다. `navigation-review/comparison-20261004-070015/`는 첫 재시험, `navigation-review/comparison-20261004-072145/`는 실패 시점 지도 기록을 추가한 다음 재시험이다. 원본 미션, 원본 실패와 성공, 설정, 지도 바이트, GT/TF, 명령 체인, CPU 자료를 삭제하거나 성공 사례로 대체하지 않았다. 두 실행 모두 종료 뒤 task-owned simulation/ROS/backend/observer 프로세스가 남지 않는 것을 확인했다.

## 발견·판단·개선

| 단계 | 근거와 문제 | 구현 및 검증 | 현재 한계 |
|---|---|---|---|
| 센서 초기화 | RGBD-only profile만으로는 LiDAR/CameraInfo가 빠져 수동 bridge에 의존했다. | `lidar_rgbd_nav` profile과 CameraInfo bridge를 추가. 수동 bridge 없는 cold start에서 AMCL/Nav2 active와 43 미션 성공. | 실제 장비 센서 교정은 별도다. |
| 관측 지도 | static map 내용 변경이 무시되거나 변경 geometry의 관측을 재해석할 위험이 있었다. | 동일 geometry는 map identity를 갱신하고 관측을 유지. geometry 변경은 disabled fault로 막으며 re-enable로 해제할 수 없다. pure/ROS 음성 검증. | chair 제거는 센서 clearing 근거가 필요하다. 관측을 지워 경로를 만들지 않는다. |
| 경로 검색 자원 | fallback이 공유 costmap mutex를 오래 잡고 선택적 clearance마다 별도 최대 예산을 사용했다. | private snapshot에서 검색하고 총 예산을 공유. 마지막 live 지도에서 padded body sweep을 다시 검사. 동시 map update·시간 예산·새 장애물 veto 테스트. | Nav2 시간 검사는 주기적이므로 soft deadline. failure 기록 I/O는 예산 이후 수행된다. |
| 시작 자세 이산화 | 실제 자세는 free인데 가까운 lattice heading으로 snap하면 몸체 모서리가 장애물에 닿아 `Starting point in lethal space`가 발생했다. | 가까운 heading 순서로 seed를 찾고 실제 자세→seed의 전체 몸체 연결을 검사. 실제 pose와 연결을 반환 경로에 유지. 실제 점유 pose의 탈출은 거부. 실패 지도 및 GT에서 EMPTY→SUCCESS 재현. | 최신 실제 실행에서 13번 도착 성공, 복귀 실패. 안전한 연결이 없는 실제 시작점은 거부한다. |
| 홀로노믹 주행 | 직진 방향과 몸체 yaw가 불필요하게 결합되고 작은 목표 오차에서 속도 격자가 정지를 선호했다. | body yaw를 유지하는 direct path, PathOrientation critic, Twirling penalty, zero-yaw 미세 이동 후보. 기존 acceleration rollout/몸체 검사/가드는 유지. 43 GT 도착 오차 개선. | 13번 중간 구간에서는 DWB 후보 523개가 모두 거부됐다. local map/평가 시점의 첫 실패 원인은 추가 검증이 필요하다. |
| 예측 시간과 발행 명령 | 실제 sim 20ms 출발 가속도 약0.9~1.0m/s² vs 설정0.4. 기존 발행 step은 rollout 간격50ms를 사용. | `use_elapsed_command_time=true`로 실제 ROS control delta를 첫 rollout에 반영하고 horizon을 보존. startup/reset/pause/backward/long gap와 braking 테스트. | 실제 odometry·명령 가속도와 downstream response를 재측정해야 한다. |
| 제어 구간 충돌 | 동일 지도에서11ms/0.629mm/0.091° 뒤 body-free→collision. 직전 issued command의3 stock-valid 후보도 중간 자세에서 collision. | stock score 뒤 endpoint convex hull + rotating-corner/center-curvature bounds + full occupied cell SAT veto. empty-box prefix certificate로 계산 최적화. saved regression/rotation/unknown/interior/cached-map change 검증. | full-cell spatial semantics가 더 보수적이다. route availability·tracking·runtime latency와 13 완전 복귀는 미검증. |
| 명령·안전 체인 | 비영 운동은 많은 swept-body 검사로 CPU가 증가했다. | 반복 OBB 생성과 SAT 축 연산 최적화. 저장 fixture에서 결정·속도·hit index 동일. 음성 ROS 검증에서 장애물/소스 유실/clock pause/취소에 정지. | 관측 voxel만 검사하며 unknown 공간이 free라고 인증하지 않는다. pan 방향 전환·fresh depth 대기가 상당하다. |
| 계획 실패 처리 | 출발 전 최초 ComputePathToPose가 RecoveryNode 밖에 있어 한 번 실패하면 즉시 미션 종료했다. | 최초 계획에도 initial+3회 한정 재시도와 0.2 s backoff. 실제 shipped XML와 Nav2 RecoveryNode로 일시 실패/영구 실패/대기 중 halt를 검증. 관측 map clearing 없음. | 정상 계획/중간 주행의 충돌을 재시도로 숨기지 않는다. |
| 도착·작업 전환 | Nav2 성공만으로 원래 요청한 도착 pose 도달을 보장하지 못했다. | 신선한 stopped odometry와 map→base TF, 원래 목표 오차를 독립 확인한 뒤 작업으로 전환. 위치 8 cm·yaw 0.15 rad. 거짓 성공/과거 odometry 음성 ROS 검증 및 43 실제 통과. | TF 정확도 자체의 확률적 보장은 아니다. GT와 구분한다. |
| 목표 생성 | chair 상태·여유·localization 불확실성을 구분하지 않고 가까운 목표를 선택할 수 있었다. | reference world로 제거된 chair 방향을 보존하되 충돌체로 추가하지 않음. localization margin 명시/검증. run-local 43·13 목표를 사용. | margin 10 cm는 시뮬레이션 가정이며 실제 calibration으로 확정한 값이 아니다. |

## 43번 개선 전후

같은 의자 제거/목표 조건에서 이전 성공 도착의 GT 위치 오차 12.84 cm·yaw 오차 4.57°가 첫 개선 재시험에서 2.99 cm·0.55°가 됐다. 다음 재시험도 도착·복귀 SUCCESS다. 이전 성공은 책상 가까운 실패 정지 지점에서 재출발했고 개선 실행은 home에서 출발했으므로 벽시계 시간·경로 길이·회전량을 통제된 A/B 효과로 해석하지 않는다. 이전 GUI와 현재 headless의 자원 차이도 분리한다. 정확한 수치는 각 `comparison.json`과 mission trace를 따른다.

43번의 지정 목표 오차는 줄었지만 첫 재시험의 책상 앞면 간격은 16.88→20.70 cm로 늘었다. 지정 pose 정확도와 책상에 더 가까이 접근하는 것은 별도 목표다. 문서에서 이를 혼동하지 않는다.

## 13번 실패 지도에서 확인한 시작 pose 문제

두 번째 재시험은 초기 계획에 성공(추가 clearance 5 cm, 58,821 iterations, 검색 0.464 s)했고 통로를 통과했다. 그러나 도착 전 DWB가 유효한 후보를 찾지 못하고 정지했다. 뒤이은 재계획은 예산을 쓰기도 전에 세 clearance 모두 `Starting point in lethal space`로 거부했다. 기록된 소요 0.004 s, iterations=0, budget_expired=false이므로 이 거부는 CPU 탐색 예산 초과가 아니다.

- 실제 요청 시작 TF: `(-0.810665, 1.628566, yaw=0.335123)`.
- snap cell center: `(-0.825000, 1.625000)`, closest lattice yaw `0.463648`.
- heading 차이 약 7.36°. 몸체 모서리 위치가 달라진다.
- 동일한 snapshot에서 C++ full padded-body 검사: 원래 pose PASS, 가장 가까운 snap pose FAIL, 원래 pose→yaw 0 seed 연결 PASS.
- 현재 costmap과 Gazebo GT 2D collision projection 모두 같은 판정이다. 따라서 **이 재계획 거부는 추가로 생성된 가짜 장애물만의 문제로 설명되지 않는다.**
- 수정 전 같은 원래 start/goal은 EMPTY였다. 수정 후에는 원래 pose를 바꾸지 않고 안전한 연결이 포함된 46-pose 경로(1.583 m)를 반환하며 요청 endpoint 위치·yaw 오차는 0이다. 모든 연결과 경로는 그대로 body sweep 검사와 최신 map veto를 통과한다.

Nav2 1.1.20의 lattice는 primitive heading 목록에서 가장 가까운 각도를 고르고 각 heading의 footprint collision을 검사한다. ([NodeLattice 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/src/node_lattice.cpp), [GridCollisionChecker 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/src/collision_checker.cpp)) 이번 수정은 실제 자세를 free 공간으로 이동시켜 가정하는 방식이 아니라 실제 자세에서 이산 seed까지 연결 자체를 검사하는 방식이다.

증거: 두 번째 실행의 `failed-plan/failure.json`, `search.bin`, `live.bin`, `*-results.csv`, `*-after-results.csv`, `*.validation.csv`, `failure-13/failed-body-overlap.png`. GT의 2D projection은 높이별 실제 3D contact test를 대신하지 않는다.

## 검증과 남은 완료 조건

관련 navigation selected build와 package tests를 실행한다. 새 경계 검증은 실제 shipped BT의 유한 재시도·halt, 원래 pose는 free/closest snap은 blocked인 출발점의 안전한 연결, 실제 occupied pose 탈출 거부를 포함한다. 기존 검색 중 map update, live 장애물 veto, 목표 정확도, generator braking 및 guard core 검증도 유지한다. 최신 결과는 `/tmp/cleany-start-seed-build.log`, `/tmp/cleany-start-seed-tests.log` 및 colcon test-result를 따른다. 전체 workspace의 과거 unrelated handeye ChArUco 실패를 이번 변경이 해결했다는 주장은 하지 않는다.

완료 전에는 수정한 seed/초기 retry가 실제 Nav2에서 로드된 상태로 43→13 도착·복귀를 확인해야 한다. 13번의 **최초 컨트롤러 후보 거부**가 계속되면 첫 실패 시점 local costmap, evaluation, wheel/GT/TF 및 명령 체인을 함께 보존해 재계획 문제와 분리해야 한다. 현재 기록만으로 그 최초 거부 원인을 확정하지 않는다.

추가 품질 과제는 camera bearing 변경/관측 대기의 안정화, 반복 실행에서 localization·persistent observation 누적 오차, 동일 출발/GUI/지도 조건의 반복 성능 비교다. 이번 구현에서 heartbeat timeout이나 obstacle/footprint/sensor freshness 기준을 완화해 성공을 만들지 않았다. 물리 장비에서 정지·센서 유실·조작 접근 검증은 별도의 증거가 필요하다.


## 최신 43→13 재검증 결과

43번은 책상 앞 도착과 home 복귀 모두 성공했다. 13번은 책상 앞 도착 및 독립 도착 검증을 통과했지만 복귀 중 실패했다. 따라서 도착 정확도는 개선됐으나 전체 미션 안정성 개선은 확인되지 않았다.

| 자리 | GT 위치 오차: 이전 → 현재 | GT yaw 오차 | 책상 앞면·몸체 간격 | 현재 결과 |
|---|---|---|---|---|
| 43 | 12.84 → 4.07 cm | 4.57 → 0.75° | 16.88 → 19.05 cm | 도착·복귀 성공 |
| 13 | 10.19 → 4.10 cm | 6.41 → 4.56° | 26.19 → 20.40 cm | 도착 성공, 복귀 실패 |

13번 목적지까지 벽시계 주행 시간은 236.3 → 288.7초로 약 22% 증가했다. 43번은 이전 출발점이 책상 근처이고 이번은 home이므로 시간 비교가 공정하지 않다. 이전은 GUI, 현재는 headless이며 시작 자세와 관측 누적도 달라 통제된 성능 A/B가 아니다. 한 번씩의 Gazebo 실행 결과이며 청소는 mock이다.

13번 복귀 실패 당시 컨트롤러가 유효한 trajectory를 찾지 못했고, 이어진 재계획은 실제 시작 pose에서 안전한 lattice seed 연결을 찾지 못했다. 세 clearance 시도 모두 iterations=0, budget_expired=false로 검색 시간 부족에 의한 실패는 아니다. 현재 자료로 최초 trajectory 거부의 근본 원인을 확정하지 않는다.

원본 comparison.json, 미션 trace, 명령 체인, controller-failure-1의 local/global costmap 및 evaluation, failed-plan의 검색 snapshot·live map을 보존했다. 테스트 이후 시뮬레이션·navigation·mission·backend·관측 프로세스를 종료했다. shutdown.json은 종료 검증을 기록한다.


## 복귀 실패 후속 판정과 동기화된 controller 진단

이 실행의 frozen search costmap과 Nav2 full padded-body checker로 원래 재계획 시작 pose를 다시 검사하면 collision-free=false다. 같은 world.sdf에서 만든 Gazebo GT 2D collision projection도 동일하다. cell center가 몸체 내부에 없다는 단순 검사는 free였지만, Nav2 몸체 경계 raster 검사는 거부했다. 따라서 cell center 검사만으로 실제 충돌 여유를 주장할 수 없으며, GT 2D 투영은 실제 높이별 물리 접촉을 입증하지 않는다. 이번 거부는 추가 costmap 장애물만으로 설명할 수 없다.

직전 기록에는 시간 정합성 한계가 있다. `/evaluation`은 sim 239.607이고 유효 187 / 거부 336개로 all-rejected 주기가 아니다. 서비스로 받은 local/global costmap은 239.811이다. `map<-base_link`의 latest common stamp는 234.171로 wheel stamp 239.649보다 과거라 현재 자세로 쓰면 안 된다. wheel pose와 map<-odom을 합성한 분석은 별도로 기록했지만, 이것도 동일 시점 센서 원본/3D contact 증명은 아니다. 직전 평가의 best trajectory를 뒤에 받은 지도에서 검사하면 거부된다. 시간차로 지도·pose가 달라졌을 수 있으므로 이를 controller의 충돌 검사 결함이라고 단정하지 않는다.

이를 해결하기 위해 production omni profile에 ObservedFootprintCritic을 추가했다. 기존 Nav2 ObstacleFootprintCritic의 점수와 예외를 그대로 위임하고, 모든 후보가 이 critic에서 거부된 첫 cycle의 정확한 local costmap/pose/velocity/footprint/path/reasons/첫 거부 trajectory를 DWB mutex가 잡힌 failure debrief에서 저장한다. 이후 실패 cycle은 같은 episode의 onset을 덮어쓰지 않는다. 정상 후보가 한 개라도 존재하면 episode가 끝난다. 다른 critic의 실패는 debug_trajectory_details 로그로 구분한다. 캡처를 켰을 때 failure-response와 map-update 지연이 발생할 수 있어 성능 측정에서는 끈다.

selected navigation build 성공. navigation package test 결과: 102 tests, 0 errors/failures/skips. 저장 실패가 충돌 판정을 바꾸지 않음, stock score/rejection 동일, mixed valid cycle은 실패로 기록하지 않음, onset 유지/새 episode 기록을 검증했다. `make test-navigation`의 마지막 전체 build-directory 요약은 기존 unrelated handeye ChArUco 실패까지 포함해 exit 1이었다. 이번 패키지의 current selected 결과는 별도로 확인했다. 새 plugin을 실제 Gazebo에서 로드하고 13 복귀 첫 실패/성공을 확인하는 검증은 아직 남아 있다.

시뮬레이션은 재실행하지 않았으며 종료 상태를 유지했다. 목표는 active로 남긴다. 도착 정확도 개선은 실제로 확인됐지만 13 전체 미션 안정성은 미완료다.


## 정확한 controller 실패 재현과 가속도·방향 연속성 수정

The preceding diagnostic run (comparison-20261004-082558) captured two exact controller failure cycles. Stock Nav2 replay reproduces all 523 rejections in each: first the current body is free but every acceleration/braking rollout intersects a lethal cell at 0.10–0.15 seconds; later the current body itself is already rejected. Unrestricted publication of the sampled velocity target differed from the acceleration-limited rollout. The measured onset and source mechanism are documented in that run's RETURN_DIAGNOSIS.md.

This run used `limit_vel_cmd_in_traj=true`, verified in the actual loaded source-runtime-parameters.yaml. 43 succeeded with 4.93 cm GT arrival error, but took 342.46 wall seconds for the full mission. 13 did not reach its target and failed at 360.82 seconds. It experienced repeated Failed-to-make-progress recoveries, followed by the mission's navigation deadline/cancellation. There were zero all-obstacle-rejected cycles. This does not prove collision reliability fixed; it demonstrates a separate progress regression and is retained as FAILED evidence.

In the slow part of 13, the pan target repeatedly alternated between 0 and approximately -π/4, with TURNING_PAN/WAIT_FRESH_DEPTH dominating the sampled gate reasons. Camera waiting stops the body, reducing measured velocity back toward zero; the next acceleration-limited proposal can again change bearing and restart camera alignment. This feedback can consume the navigation deadline even when geometric paths and collision-valid trajectories exist. Neither the navigation deadline nor sensor freshness/alignment criteria were loosened.

After this run was stopped, CommandBearingCritic was added as a soft continuity cost, with scale 2.0 and linear fade inside 0.3 m of the goal. It preserves zero-command history, resets on new paths, and gives finite costs to lateral/reverse directions. It does not veto a travel direction or alter obstacle scoring. The parameter values need actual simulation performance verification; no full-mission success is claimed for the new critic. Its rationale, interface, boundary tests and limitations are in the package README and source.

Selected build and current navigation test result: 104 tests, 0 errors/failures/skips. `make test-navigation` still includes a historical unrelated handeye ChArUco failure in its final whole-build-directory summary; the current selected navigation test result was inspected separately. Source and runtime stages differ intentionally: source-runtime-parameters.yaml is the profile used by the failed limit-only run, while bearing-source-hashes.json records the subsequent unverified continuity change.

Simulation, navigation, mission, backend and task-owned observers were stopped. shutdown.json confirms no owned processes remain. Goal remains active: next verify that the combined acceleration/continuity settings load and complete 43→13 arrival plus home return, preserving exact failure records if not.


## 결합 프로필 실주행 및 시뮬레이터 처리량 경계

Runtime confirmed limit_vel_cmd_in_traj=true and CommandBearingCritic scale 2.0 / near_goal_distance 0.3, with the critic actually loaded by Nav2. Source-runtime-parameters.yaml and navigation log offsets retain provenance.

43: full mission SUCCESS, 183.41 wall seconds; GT arrival error 7.31 cm and yaw error 2.74°. Compared to the preceding limit-only run (342.46 seconds), this run was shorter, but map observations/localization differed; this is not a controlled performance A/B.

13: desk arrival and independent arrival gate succeeded at approximately 323 wall seconds, GT position error 7.72 cm and desk-front clearance 23.04 cm. Return crossed the former collision-failure region and continued into the home corridor. It ended at approximately (-1.968, -1.437), not home: total 685.72 seconds and return wall deadline approximately 360 seconds. Outcome remains FAILED/TIMEOUT. No all-obstacle-rejected cycle or Failed-to-make-progress error occurred in this run. Successful desk arrival must not be represented as full mission success.

The configured physics real_time_factor is 1.0, max_step_size 0.001. Actual sim/wall clock ratios are in clock-analysis.json. An earlier recorded same-run idle/active clock comparison was 0.3967 / 0.3969 respectively. Thus the slow simulator was already slow at idle; mission load alone cannot be assigned as the cause. Recorded active task CPU averaged about 390% (100%=one core), with Gazebo around 152%, depth_clearance 54%, obstacle_memory 43%, controller 19% and voxel_guard 18%. Matched idle CPU data are missing, so these values do not quantify an idle-to-navigation CPU increase or a before/after CPU effect. Container CPU quota/cpuset are unrestricted. Requested RTF is not measured RTF.

A subsequent source change exposes optional physics_max_step_size on Fortress/Harmonic. Empty preserves the supplied SDF; an explicit finite value in (0, 0.01] changes only a materialized copy. Geometry, source file and requested real_time_factor remain unchanged. This allows a 2 ms integration-step experiment without changing motion/collision limits, sensor rates or mission deadlines. The experiment has NOT yet run: timestep affects integration fidelity, and neither a speedup nor stable contact dynamics is claimed. Source helper tests passed 8/8; selected simulator build succeeded; installed Fortress --show-args exposes the option. Current navigation tests remain 104/104 from the preceding unchanged navigation build.

All task runtime/backend/observer groups were stopped. shutdown.json records the final audit. Goal remains active: investigate simulator throughput with matched idle/moving clocks and CPU, stationary drift/3D attitude and actual mission outcomes under an explicit 2 ms profile, then complete 43→13 home return before claiming the integrated goal achieved. Mission timeout semantics were not changed.


## 최신 2ms 43 / 13 비교 검증

comparison-20261004-093808의 RESULTS.md와 comparison.json에 결과를 보존했다. 43은 211.8초에 FAILED (GT 목표 거리 6.25cm, 추정 거리 5.68cm, 진행 불가). 13은 43 실패 후 별도 실행하여 255.2초에 FAILED (책상 미도착, 전체 523 controller trajectory 거부). 이번 실행에서는 두 자리 모두 도착 성공과 홈 복귀를 입증하지 못했으며, 개선 전 SUCCESS 기록 대비 신뢰성 개선을 주장하지 않는다. 출발 위치·GUI·물리 timestep·관측 지도 조건이 달라 시간은 통제된 A/B가 아니다. 실패 진단과 마지막 지도, 실제 runtime world/parameters, 미션 trace를 보존했다. 동일 실행 idle CPU 429.3%, seat43 active 449.5% (100%=one core). 시뮬레이션·navigation·mission·backend·task observers를 종료했고 shutdown.json의 remaining은 빈 배열이다.


## 43 근접 제어 점수 수정 / 13 실패 원인 판정 경계

정확한 093808 controller-cycle-1을 stock Nav2 footprint critic과 설치된 HolonomicGenerator로 재생했으며 body_valid=false, 523/523 후보 first_invalid_index=0이었다. 첫 캡처 시점의 몸체 자세 자체가 costmap에서 거부된다. 이전 cycle의 지도/추정 자세가 없으므로 지도 변화·motion tracking·localization 중 무엇이 그 상태를 만들었는지는 아직 확정할 수 없다. 이를 해소하기 위해 ObservedFootprintCritic은 직전 debrief cycle을 raw grid/path/pose/velocity/command/stamp로 메모리에 한 개만 보관하고 첫 실패 manifest에 previous_cycle로 함께 기록한다. 새로운 path에서는 prior/latch를 reset한다. 다른 critic의 성공이나 downstream 명령을 입증하는 기록은 아니며 capture 활성화의 copying/I/O overhead는 성능 실험에서 제외해야 한다.

43 정체는 마지막 추정 목표 거리 5.68cm에서 발생했다. 정확한 nominal evaluation을 저장하지 않아 그 실제 cycle의 최적 점수를 단정할 수는 없다. 그러나 stock GoalDist를 사용한 regression에서 goal과 stop/closing endpoint가 같은 5cm cell에 있으면 목표 거리 5.66cm로 goal tolerance 밖이어도 모두 score=0이었다. 기존 bearing preference는 closing만 벌점하여 이 plateau에서 stop을 선호할 수 있다. PreciseGoalDistCritic은 stock 지도 guidance/rejection에 실제 endpoint 거리/resolution 항을 목표 0.3m 안에서만 추가한다. CommandBearing free_goal_distance=0.1m에서는 penalty가 0이며 0.3m까지 선형 증가한다. 도착 오차·footprint·pan freshness·voxel safety·mission deadlines는 변경하지 않았다. Regression은 closing 우선, far stock 동등, off-grid/empty/nonfinite 거부, 직전 지도 immutable 보존/새 plan reset을 검증한다.

최종 selected navigation build 및 test-result: 106 tests, 0 errors/failures/skips. git diff --check 통과. 이번 변경 이후 43/13 Gazebo 도착/복귀 검증은 아직 실행하지 않았다. 시뮬레이션은 종료 상태이고 goal은 active로 유지한다.


## 100444 실제 검증 및 복귀 실패 onset 재현

기본 1ms에서 수정된 precise-goal/bearing 설정이 로드된 것을 확인했다. 43 전체 SUCCESS (GT arrival 5.65cm, yaw 2.01°, total 242.8wall), 13 desk arrival SUCCESS (5.19cm, yaw4.88°) 후 return FAILED (total469.7wall). RESULTS.md와 comparison.json에 비교/한계를 기록했다. 특히 첫 all-obstacle-rejected cycle의 previous_cycle와 current 비용지도는 identical, 11ms에서0.629mm/0.001583rad 움직여 body-free가body-invalid로 변했다. 직전 issued velocity와 일치하는 6target 중 stock-valid3개는 모두 첫 .05s pose 구간의 dense interpolation에서 collision을 보인다 (첫 fraction.11,약5.5ms). 이는 이전 후보의 temporal collision-sampling gap을 재현한 구체적 반례다. 별도의 stamped GT odom에서 시작 평균 가속도0.9~1.0m/s² vs설정0.4도 측정했다. Limit publication이 실제 control sim-time delta가 아닌 trajectory 첫 step에 묶인 점을 개선해야 한다. 이 실행 이후 generator/critic source는 아직 수정하지 않았다. 모든 task runtime/observers 종료 remaining=[] 확인. Goal active: immediate command sweep와 actual command timebase를 수정하고 real43/13 complete return을 다시 검증해야 한다.


## elapsed command / conservative swept footprint 구현 검증

`FollowPath.use_elapsed_command_time=true`는 limit_vel_cmd_in_traj의 첫 step을 실제 ROS control delta로 분할한다. 첫 iteration·reset·clock pause/backward는 zero budget, 큰 gap은 기존 first interval로 cap하며 전체 rollout horizon을 유지한다. 기존 legacy 재생은 별도의 명시적 false 파라미터 파일을 사용해 기록 당시 semantics를 보존했다.

`SweptFootprintCritic`은 stock score/rejection 이후 모든 adjacent pose의 endpoint footprint hull에 rotating-vertex second-derivative bound와 held body-twist center curvature bound를 더하고 lethal/unknown cell square 전체와 SAT 교차 검사를 한다. 직전 기록 pose는 stock boundary raster에서는 free지만 full-cell overlap 모델에서는 이미 invalid라, 이번 veto가 temporal 보간만의 영향이라고 주장하지 않는다. 공간 모델을 함께 강화한다. empty-region certificate는 DWB map mutex가 held인 prepare에서 occupancy prefix를 매cycle 새로 만들며, nonempty bbox는 full hull/SAT를 수행한다. numerical margin은 exact cell-touch에서 rounding으로 인한 누락을 방지한다.

최종 navigation tests111개 통과 (clock delta와 frozen failure/rotation bowed arc/midpoint collision/unknown interior/clear map score/cache update 등), selected build와 git diff --check 통과. offline10-round same-candidate benchmark는 prepare+523critic scores만 포함한다. 초기 full-SAT empty map98.66ms는 제어 예산에 부적합하여 empty-box certificate로 최적화했다. 최종 측정값은 sweep-verification JSON/README에 기록하며 실제 runtime CPU 효과나 worst-case latency로 주장하지 않는다. 이 source change 이후 시뮬레이션 재실행은 아직 안 했으며13 full return/acceleration/live latency 검증이 남아 goal active다.


## 104752 검증 및 방향 유지 가감속

실행 결과와 CPU/clock/명령 telemetry는 해당 RESULTS.md/runtime-analysis.json에 보존했다.13 outbound TIMEOUT 실패를 유지했다. 같은 target의 acceleration/deceleration동안 XY별독립clipping이 bearing을 바꾸는 반례를 공통 interpolation fraction으로 수정했다. std per-component projectVelocity의허용delta중최소fraction을 사용하므로 각각의기존budget을넘지않고 angularmotion은독립적이다. publishfirststep과future rollout모두같은projection을사용한다. Legacy 재생은 preserve_translation_direction=false와use_elapsed_command_time=false를명시한다. Regression은 negative/lateral fixed-target 출발·정지 heading유지, 각각의accbudget, reversalsegment,clockpause/reset/backward/cap을검증했다.112tests0failures,selectedbuild 및diff check통과. CommandBearing reset은변경하지않았다.20Hzreceivedplan은pruning republication이라reset frequency로쓰지않는다.

MecanumDrive 실제SDF±2 vs Nav2 .4/.6 mismatch를기록했다. installed6.18.0의지원tag를확인하지않고 최신upstreambranch기능을가정하지않는다. 이번변경은planner/model의ramp방향문제를개선했지만actuator가속도보증은아니다. 모든taskprocess종료,remaining=[];XYramp후실주행과simulationactuator/model一致검증,13completehome return이남아있다.

## 종료 후 병목 분석과 depth 자기 필터 개선

직전 goal turn은 실제 43 전체 성공/13 TIMEOUT 및 종료 확인으로 진전이었다. 새 실행은 시작하지 않았다. 원본 기록을 30초 구간으로 분석했다. 미션 전 25초 구간에서 GT 좌표는 일정하고 cmd_vel은 모두 0이었다. 이 warm idle의 시험 소유 CPU 합은 평균 373.79%(100%=1 core)였다. depth_clearance 평균52.69%, controller_server8.57%. 13의 180~210초 구간에서는 controller_server96.23%, 소유 합464.38%였다. Observer/다른 앱은 합계에서 제외되므로 전체 컴퓨터 CPU 수치가 아니다.

13의 330~360초 구간은 TURNING_PAN14.61초 + WAIT_FRESH_DEPTH4.48초, 관측 target 변화>0.04rad 93회, 샘플 GT 이동21.18cm다. 현재 source는 매 gate step에서 DWB 명령의 atan2로 목표각을 갱신한다. 반복 목표 변경과 카메라 대기는 진행 저하의 근거다. 하지만 이미 실패한 actual-pan critic을 다시 도입하거나 gate 각도/센서 freshness 기준을 느슨하게 하지 않았다. 13 TIMEOUT의 단일 원인 또는 최종 해결이라고 주장하지 않는다.

별도 cProfile에서 실제 park profile의59개 자기 형상에8480개 합성 ray를 처리하는 self_observation_mask가20회에0.725초를 썼고, numpy reduction이0.308초를 차지했다. 동일한 닫힌 선분 slab 교차 계산을 축별 enter/leave 누적으로 바꿔 N×3 임시 배열·축약을 줄였다. 기존/새 구현을 교대로20개 합성 ray 집합에 적용한 결과 mask는 모두 같았고 중앙 처리 시간36.12→23.19ms(약35.8% 감소)였다. 실제 depth frame 또는 live node CPU의 개선 측정은 아니다. sensor FPS/stride/range, filtering margin, touch/parallel/invalid 처리와 safety gate 기준은 변경하지 않았다.

관련 pure core + ROS transport tests19개 통과. 회전 OBB의 독립 scalar ray 판정, 비유한 값, 빈 cloud, 표면 접촉, 박스 내부 센서를 포함한다. benchmark/분석 스크립트와 JSON은 navigation-review에 보존했다. 이 변경 후 43/13 실제 주행·제동 응답·CPU 재시험은 아직 하지 않았다. 시뮬레이션은 종료 상태를 유지한다.

추가 종료 감사: container exec가 launcher와 다른 PGID로 만든 observe-chain.py PID405522를 발견했다. 최초 PGID 기반 remaining=[]만으로는 observer 종료를 증명하지 못했다. 기존 감사 JSON을 보존하고, 현재 run의 정확한 observer 파일 argv와 checkout cwd를 함께 검사하도록 stop helper를 수정했다. 해당 observer를 SIGINT로 종료했고 재검사 결과 current-run observer 없음 / shutdown remaining=[]다.

## 135422 재주행 및 종료 후 패딩 기준 수정

## 종료 후 분석: 방향 고정 후보의 패딩 기준 불일치

저장한 global map(stamp171.344)에 13번의 두 경로(stamp110.253/151.089)를 같은 C++ swept checker로 재생했다. 추가 5cm/2.5cm 패딩에서는 통로의 코너에서 모두 거부됐지만, body+설정 패딩 2cm에서는 둘 다 전체 연속 sweep을 통과했다. map은 계획보다 뒤 시점이므로 당시 거부 원인을 확정하는 증거는 아니다.

코드에서는 추가 탐색 margin이 fixed-heading 후보의 수락 조건에만 적용되고, 기존 회전 route는 기본 required footprint로 수락됐다. fixed-heading 수락도 기본 required footprint(설정된 costmap 패딩 포함)로 맞췄다. snapshot/live 두 번의 full-cell sweep, unknown/lethal/bounds veto와 shared deadline/cancel은 유지한다. 추가 lattice search margin은 그대로다.

빌드 통과, navigation 패키지 118 tests / 0 errors / 0 failures / 0 skipped. 추가 회귀는 optional search padding으로만 거부되는 경로, required padding에만 닿는 lethal/unknown의 veto를 구분한다. 최초 테스트는 unpadded body가 셀 경계에 정확히 닿는 좌표여서 거부됐고, body 경계에서 1cm 떨어진 좌표로 fixture를 수정해 패딩 차이를 검증했다. 코드를 느슨하게 바꾸지 않았다.

수정 후 실주행은 하지 않았다. 13 완전 도착/복귀, actual fixed-heading 선택 및 downstream 동작 검증이 남는다. 시뮬레이션은 종료 상태다. fixed-heading-replay*.json/cpp, fixed-padding-*.log/json 참고.

## 방향 고정 XY 탐색의 offline 전체 planner 검증
# 방향 고정 XY 탐색 구현 및 offline planner 재생

이전 패딩 수정 뒤 planner 전체를 재생한 결과 첫 경로는 여전히 회전 route(4.095rad)를 선택했다. 단순히 이미 찾은 XY를 재정렬하는 방식만으로는 해결되지 않는 반례다. 두 번째만 기존 XY 방향 고정(0.00633rad)에 성공했다.

새 checkedFixedHeadingGridPath는 body yaw를 시작/목표 각도로 고정하고 XY를 8-neighbor A*로 탐색한다. 실제 body+configured padding으로 모든 이동 edge와 정확한 endpoint 연결/회전을 full-cell swept checker로 검증한다. center inflation은 기존 cost_penalty를 사용한다. private snapshot에서 탐색하고 반환 직전 최신 live map에서도 전체 sweep을 검사한다. 각 heading에 기존 max_planning_time의1/4을 배정하고 rotating lattice fallback을 보존한다. holonomic_direct_path=false는 기존 경로를 유지한다. unknown/lethal/map bounds와 required padding, downstream safety 조건은 유지한다.

|같은 frozen map/endpoint|rotating lattice (holonomic false)|새 fixed-heading XY (holonomic true)|
|---|---:|---:|
|경로1 누적 body yaw|4.09504 rad|0.02615 rad|
|경로1 길이|8.542 m|8.661 m|
|경로1 offline 계산 시간|408.83 ms|40.82 ms|
|경로2 누적 body yaw|3.14793 rad|0.00633 rad|
|경로2 길이|3.425 m|3.640 m|
|경로2 offline 계산 시간|89.35 ms|13.15 ms|

네 실행 모두 정확한 시작/목표 pose와 전체 swept-body 검증을 통과하고 각2s 예산 이내였다. 새 branch의 실제 선택 로그를 저장했다. 지도는 계획 당시가 아닌 이후stamp171.344 서비스 snapshot이므로 historical 실패 재현 또는 live 주행/CPU/최악지연 개선의 증거가 아니다. fixture는 live plugin 대신 이미 rasterized된 map byte를 Costmap2DROS에 주입한다. 동적 장애물/센서 갱신과 pan/DWB 추종은 포함하지 않는다.

빌드 및 navigation120 tests 통과. 회귀는 full-body detour, cancellation, unknown complete barrier, endpoint rotation collision, angle wrapping/invalid inputs, 반드시 body를 회전해야 통과하는 corridor에서 fixed-yaw search의 실패를 검증한다. 기존 snapshot/shared-budget/live-update tests도 통과했다.

새 실주행은 하지 않았다. 시뮬레이션은 종료 상태. 종료 재감사에서 과거06:29 localization lifecycle manager PID126594를 발견했고, 이전세션PGID126371/cwd/domain224/정확한argv로 소유권을 확인해 INT→TERM→KILL로 종료했다. stop helper는 최신run PGID 외 integration cwd/domain224 잔여를 읽기 전용으로 보고하고 남아 있으면 종료검증을 실패시키도록 개선했다. 관련 cleanup/shutdown JSON 확인.

## 142512 실제 시험 및 bounded pan position branch 수정
# 방향 고정 XY 탐색 실제 43→13 주행 결과

GUI 없이 43 SUCCESS 후13을 순차 실행. readiness(필수 lifecycle6개와 sensor feedback) 통과, 실제 loaded planner .so SHA가 offline 검증 build와 일치했다. Gate의 position-branch 수정은 이 실행 이후이므로 아래 결과에는 포함되지 않는다.

|항목|직전135422 실행|현재142512 실행|
|---|---|---|
|43 전체 결과|SUCCESS (263.7s)|SUCCESS (305.3s)|
|43 도착 GT 위치 오차|6.79|4.97|
|43 도착 GT yaw 오차|0.17|0.48|
|43 책상 앞 간격 cm|21.69|19.13|
|13 전체 결과|FAILED (361.0s)|FAILED (652.6s)|
|13 도착 GT 위치 오차|미도착|3.45|
|13 도착 GT yaw 오차|미도착|1.58|
|13 책상 앞 간격 cm|미도착|22.89|

43 도착+복귀 SUCCESS. 13은289.76s outbound 후 도착 GT3.45cm/yaw1.58°로 WORKING에 진입하고292.8s부터 복귀했으나652.6s TIMEOUT. 최종 출발점 GT거리1.726m; 목표값/속도/안전/timing deadline을 완화해 성공 처리하지 않았다. Navigation OK, return CANCELLED라는 terminal 원문을 보존한다.

13 outbound/return에서 실제 fixed-heading XY search 선택 로그가 각각174/175poses로 남았다. outbound 계획 길이8.659m, 누적yaw.040rad; 이전첫 계획은3.168rad였다. 실제13 전체 GT누적yaw.746rad로 방향 유지가 측정됐지만 complete return 실패이므로 전체 개선 성공을 주장할 수 없다.43 direct route는 기존 구현이며 현재소요시간은 직전보다 길었다. Single trial/시작 위치·GUI·background process 차이로 baseline wall/CPU의 통제된 인과 A/B는 아니다. owned CPU는 audit observer와 다른 앱을 제외한다. warm idle 분석창 일부는 startup일 수 있어 stable idle로 단정하지 않는다.

## 복귀 병목과 후속 구현

Pan raw target +π/−π 전환12개 중7개는 동일 방위각의 범위 내 위치를 선택하면 실제joint 위치에서 훨씬 짧게 이동할 수 있었다. 예: panπ, target−3.11978은 raw6.26137rad 이동인데 실제SDF범위[-3.2,3.2] 안의+3.16341로는0.021815rad이다.나머지전환은 finite mechanical bounds 때문에 실제 긴회전이 필요할 수 있다. recorded actual-pan를 입력으로 한 setpoint counterfactual이며 전체closed-loop 시간절약이나 다음run SUCCESS는 증명하지 않는다. 모든event/제한은 pan-position-branch-analysis.json에 있다.

PanGate에 pan_min_position_rad/max_position_rad를 추가해 실제feedback에서 가장 가까운 범위 내2π-equivalent position을 명령하도록 수정했다. clipping이나 tolerance 확대를 하지 않는다. 목표방위각 표현이 불가능하면 PAN_TARGET_OUT_OF_RANGE로 zero 출력하며 joint/depth freshness를 유지한다. 기본값[-π,π]는 기존 command domain이며 physical calibration을 의미하지 않는다. run-local Gazebo 설정만 world.sdf의[-3.2,3.2]로 명시했다. preflight도 parameter service 값이 SDF한계와 일치해야 mission을 시작하도록 보강했다.

관련 pure gate12tests, navigation127tests0failures, build/diff check 통과. 동일방향+양/음 branch, 관절경계를 넘는 equivalent 거부, 정렬오차 및 old-depth 차단, unreachable/invalid limits를 검증했다. isolatedROSdomain231 nodeconstructor는 gate-only parameter yaml의 실제binding을 확인했고 executor를 spin하지 않아 메시지를발행하지 않았다. 원본rawsafety scaffold에는 runtime adapter가 변환할 nested배열이 있어 그대로RCL에 넘긴 첫 fixture는 parse실패했다; pan node설정만추출한fixture로 수정한 것이며 실제launch 실패가 아니다.

이후 시뮬레이션을 다시 실행하지 않았다. shutdown.json remaining=[] / untracked_integration_runtime_pids=[] 확인. 성공판정은 계속미충족: post-change43/13 completearrival+home return, livepan branch동작/compute/actuatorfidelity 검증이 남는다.

## 145413 재시험 실패 및 goal-heading/live validation 분리
# 145413 pan-position 수정 후 실제 주행 및 planner 후속 수정

43 SUCCESS330.64s, GT 도착4.674cm /0.223°. 13 outbound TIMEOUT361.34s, 최종GT[-1.804721,-2.123913,1.421974], 목표거리4.653m. 복귀는 시작하지 않았다. 실제GetParameters로pan[-3.2,3.2]가loaded된 readiness 통과 및source/library fingerprints를 저장했다.이번실행은후진stage에이르지않아앞선복귀±πbranch수정의성공을증명하지못한다. 목표·안전조건·timeout을완화하지않았다.

13의첫계획 fixedheading1.472336rad와재계획1.442736rad는각각현재시작heading을유지했다. 실제몸체heading도1.46→1.42로변하고panbearing전환/정지가반복됐다. GT/estimated최종yaw는1.421974/1.420287로비슷하므로원인을단순localizationyawbias로확정하지않는다. sparseXYvelocitysampling,bodyyawtracking,pan/replanfeedback은추가분석이필요하다.

## 실행 종료 후 구현 변경

Code inspection에서live-check가checkedFixedHeadingPath라는대체경로생성기를다시호출해,private search가선택한heading을시작heading으로바꿀수있음을발견했다. 입력poses를그대로검사하는sweptPathCollisionFree를분리하고snapshotcandidate의orientation을live단계에서변경하지않는다. 비유한/미정규화quaternion/frame/footprint/cancel 검증을유지한다.

Fixed-heading XY search는요청된goalheading을먼저시도하고초기회전의전체body sweep을검사한다. 안전한goalheading경로가없으면startheading과rotatinglatticefallback을기존shareddeadline안에서시도한다. replanning마다currentheading을새preferredheading으로채택하는변동을줄이려는정책이며, downstream성공은미검증이다.

128navigationtests통과,build/diffcheck통과. 회귀는최초선택heading에새장애물이생기면다른heading이통과할수있어도그경로의livevalidation을실패시키는것과invalidoriginalquaternion보존을검증한다.현재run의두시작pose를이전135422 mapbytes에넣어실제planner를실행했을때,모든interiorheading은goalπ/2와일치했다. exactendpoints/fullsweep 통과,각39.90/34.53ms; 합계plannedrotation.09846/.12806rad. 이map은현재trial의historicalmap이아니므로 실패원인재현이나live성공증거로주장하지않는다.goal-heading-offline JSON/logs/source 참조.

shutdown remaining=[] 및untracked_integration_runtime_pids=[] 확인.새수정후simulation은다시띄우지않았다. 13completearrival+home return, camera branch의실제reverse동작/compute/actuator모델검증이여전히남아있다.

## 152008 실제 주행 결과와 근거리 속도 후보 개선

151527 시작 시 map_server lifecycle 응답 시간 초과로 readiness가 실패했고 미션은 제출되지 않았다. 소유 프로세스를 종료한 뒤 새 152008 실행은 readiness를 통과했다. 시작 가용성 문제는 미해결이며 성능 실패와 구분한다.

43은 SUCCESS262.07s, GT 도착 오차5.21cm/2.11°, 책상 간격17.95cm였다. 13은 outbound257.46s 후 WORKING, GT 오차4.77cm/.52°, 책상 간격24.85cm로 도착했다. 복귀는360.24s TIMEOUT: 전체620.70s, GT home까지18.77cm, estimated home까지11.13cm 남았다. GT와 추정 위치의 차이가 있으므로 estimated arrival 판정은 GT 정확도 보증이 아니다. 이전06시 baseline은43/13 SUCCESS였으므로 현재 전체 신뢰성이 좋아졌다는 결론은 반박된다. 출발 위치/GUI/배경 부하가 달라 wall-time 인과 비교도 불가능하다.

실제13 outbound/return fixed-heading 계획 interior yaw는 요청된goal yaw와 일치했다. 마지막 복귀59초 sampled pan reason은TURNING_PAN452/WAIT_FRESH_DEPTH164/PASS517이었다. Unique ROS stamp 기준 target 변경>.04rad213회, raw jump>π0회였다. 이는 sample 통계이며 정확한 이벤트수/시간 비율이나 인과 증명은 아니다. Nav2 요청 bearing은π와2.677945rad에 빈번히 머물렀다. fine_translation_scale은 속력만 축소하므로 기존 Cartesian grid의 방향 간격을 해소하지 못한다. 이전±π branch 문제와 구분되는 근거리 방향변경 병목 가설이다. 원본near-home-analysis.json/comparison.json 보존.

후속 GoalDirectedController는 [Nav2 1.1.20 DWB](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)의computeVelocityCommands 호출 중 현재pose와global_plan 마지막goal을 같은frame으로변환해Generator에동기입력한다. 기존prepareGlobalPlan을중복실행하거나pruned path의끝을최종목표로취급하지않는다. HolonomicGenerator는반경.75m내에서정확한body-frame goal벡터를따르는zero-yaw candidate를최대3개추가한다. 속력은distance/sim_time과방향별component/XY speed한계의작은값에.25/.5/1을곱한다. 기존grid/fine후보,모든critics,전체acceleration/brakingrollout과serialsafety를유지한다. context는1회소비/reset/예외포함control-call종료에서지워지고TF실패시후보를추가하지않는다. generator기본반경0은비활성;omni profile만.75m로활성화한다. 목표위치/timeout/안전판정은완화하지않았다.

빌드및navigation131tests0errors0failures0skips,diffcheck통과. 회귀는실제마지막estimatedpose를사용한world→body벡터,가속예산과predictionhorizon,disabled/invalid/outside-radius/expired/resetcontext,forward-onlyprofile의reverse후보금지,nav2_core pluginlib load를검증한다. 이변경후Gazebo는실행하지않았으므로closed-loop정지감소/43·13completehome return/CPU개선은미검증이다.

시뮬레이션/Nav2/mission/backend/observer는계속종료상태이며152008 shutdown remaining=[] / untracked_integration_runtime_pids=[] 확인했다. 새코드와기존주행증거를혼동하지않도록implementationartifact/sourcehash와run152008 provenance를분리했다.

## 155157 후보 추가 실주행 실패와 경계 결함 수정

실험 후보를 실제로 평가했으나 43번은 336.71초에 NAVIGATION_FAIL로 종료됐고 GT 목표 잔여 거리는 55.32cm였다. 13번은 실행하지 않았다. 다른 실행의 13 결과를 현재 43 결과와 조합하지 않는다. Nav2 로그에는 네 번의 `Failed to make progress` 뒤 `Goal failed`가 기록됐다. 목표, 시간, 안전 가드와 진행 판정을 완화하지 않았다.

실제 GetParameters 응답으로 wrapper와 radius .75를 확인했고 SO, parameter 파일과 source hash를 보존했다. stamp 93.812의 525개 후보 중 마지막 두 개는 goal-directed였지만 최고 속력 후보는 누락됐다. 선정 후보 389의 total은 6.833, 추가 후보의 partial total은 9.653/8.453이었다. 탈락 후보는 short-circuit 평가가 가능하므로 완전한 점수 비교로 취급하지 않는다. 최신 표본의 Pan 상태를 적분한 값은 TURNING119.76s / WAIT_FRESH_DEPTH103.19s / PASS106.19s / WALL_TIMEOUT7.51s였다. .04rad를 넘는 목표각 변경은 1211회, raw π jump는 0회였다. 장시간 정지를 보여주는 표본이며 원인의 완전한 인과 분리 시험은 아니다.

종료 후 angle 1.417530671655 회귀 테스트가 baseline+2(actual1254) 대신 +3(expected1255)를 요구하며 실패했다. .15m/s 벡터의 norm이 반올림으로 .15000000000000002가 되면 strict limit에 거부된다. Target speed를 표현 가능한 두 단계만큼 한계 안쪽으로 이동시켜 속력 한계를 늘리지 않고 해결했다. 후보 8방향, 가속, 만료, 범위, forward-only 제약, plugin loading 및 기존 swept/gate/planner 검증을 포함해 navigation132tests/build/diffcheck가 통과했다. 이 수정이 43 실패의 주원인이거나 왕복을 해결했다는 증거는 아직 없다.

기본 omni는 DWB/radius0으로 복구했다. 실험은 `CLEANY_REVIEW_GOAL_DIRECTED=1`로 run-local profile에 명시한다. Launcher는 요청 profile을 q/omni-params.yaml로 동결하고 hash를 기록한다. Readiness는 해당 profile과 실제 controller parameter를 비교한다. 종료 감사에서 remaining=[] / untracked=[]를 확인했다. Planner exit-11은 mission 실패 후 SIGINT 정리 중 발생했으며 별도 조사가 필요하다.

## Planner 종료 오류의 원인과 수정 검증 (161106)

155157의 PID568796 core를 Ubuntu ROS 환경의 GDB로 읽었다. crash는 planner main 종료 후 `__run_exit_handlers` → `free`에서 발생했고 destructor argument는 `NodeLattice::dist_heuristic_lookup_table`이었다. 실제 link dependency에는 `libnav2_smac_planner.so`와 `libnav2_smac_planner_lattice.so`가 모두 있었다. 두 라이브러리는 같은 전역 table을 각각 정의한다. [Nav2 1.1.20 CMake](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/CMakeLists.txt)도 각각 node_lattice.cpp를 포함하고 모두 export한다.

기존 검색 회귀의 `_Exit`는 static destructor를 건너뛰었고 raw planner도 해제하지 않았다. 이를 실제 planner/costmap cleanup, ROS shutdown, `std::exit`로 바꾸자 `double free or corruption`으로 실패했다. Adapter의 ament package-wide library link를 필요한 lattice library 하나로 좁히고 exported include 경로는 유지했다. 동일 회귀는 정상 종료로 통과했다. 전체 navigation132tests/build/diff check 통과.

별도 ROSdomain231의 실제 planner_server에서 configure/cleanup 성공 후 SIGINT 종료 코드0을 확인했다. /proc maps에는 Cleany adapter와 lattice 하나만 있었다. 첫 probe의 plugins=[] YAML은 typed array로 전달되지 않아 시작 실패했으며, dummy InflationLayer를 명시한 fixture로 수정했다. 이 fixture 실패는 source planner 결함으로 집계하지 않는다. 실행 중인 Gazebo와 주행 목표는 없었다. 실제 probe와 core 분석, before/after 테스트 및 sourcehash는 `planner-shutdown-20261004-161106/`에 있다. 다른 노드 종료와 active planning cancellation은 이 증거 범위에 포함되지 않는다.

수치 후보 수정 후 첫 실제 실행161152는 controller change_state 응답 전송 timeout으로 readiness가 실패해 미션을 제출하지 않았다. INT/TERM 뒤 남은 owned lifecycle manager는 cwd/PGID/argv/domain/start ticks를 확인해 KILL했고 shutdown remaining=[] / untracked=[]를 확인했다. 161523의 한 번 새 시작은 모든 readiness 항목을 통과했으며 실험 프로필의43 미션을 실행 중이다. 최종 주행 결과는 아래 후속 기록을 따른다.

## 161523 수정 후 실제 결과

43 SUCCESS327.74s: GT arrival6.68cm /3.71deg, outbound279.35s, return46.62s. Desk clearance18.53cm. Previous152008 SUCCESS262.07s /5.21cm /2.11deg. Current327.74s is slower; a single uncontrolled run does not prove speed improvement.

13 FAILED/TIMEOUT363.93s, no WORKING and no return. Last sampled GT was1.33cm from goal, while estimated distance was7.44cm and GT-estimated discrepancy8.48cm. These are latest-received samples, not atomic or guaranteed fresh at termination. Do not call proximity successful arrival. The comparison arrival fields remain null because no formal arrival occurred.

Late owned CPU mean rose from389.6% in elapsed180–220s to868.9% in280–330s (100%=one logical core). Host sampled97% busy with23GiB available memory and no swap. The final safety state was STOP_UNAVAILABLE/Source wall-clock timeout; Pan state also WALL_TIMEOUT. Nav2 missed control rate, BT missed tick rate; Gazebo clock advanced slowly. This does not isolate a root cause or prove the candidate policy caused the overload. The trace became sparse late in the run, so standard .5s sampling assumptions do not hold there.

The directed candidate omission is fixed and live evaluations at86.786/94.285 had526 candidates, with the highest directed candidate525 selected and fully scored. This proves candidate presence/selection at those observed cycles, not full navigation reliability or less pan waiting. Existing bounds, prediction and all critics were preserved.

Planner link fix: only lattice implementation loaded in actual process; core-based duplicate-table destructor defect reproduced before, normal-exit regression passed after, and isolated planner-server configure/cleanup/SIGINT exited0. Full live shutdown log did not preserve a planner exit-code line, so do not infer graceful exit0 from process absence. Other shutdown crashes remained: collision_monitor SIGABRT and some Python nodes SIGSEGV. They are separate unresolved defects.

161152 startup failed controller lifecycle response publish; no mission submitted. Owned stalled lifecycle manager was verified then killed. One bounded fresh retry161523 passed readiness and produced the outcomes above.

All owned sim/Nav2/mission/backend/observer processes stopped; shutdown remaining=[] / untracked=[]. The crash analyzer for deliberately failing regression PID583206 was still consuming about100% CPU and4GB RSS. Its exact ABRT report/executable and parent chain were verified; only that report's analysis workers were terminated. Stored core preserved. This concurrent load is recorded, not claimed as the complete runtime failure cause. Test child now suppresses its own core generation while still executing cleanup and normal static destructors.

Default omni remains DWB/radius0. GoalDirectedController remains opt-in, and full43/13 return remains unproven.

## 기본 빌드 최적화와 같은 환경 비교 (170804)

CMake build type이 비어 있어 Cleany C++ hot loop가 -O0로 컴파일되는 것을 확인했다. single-config의 미지정 build만 RelWithDebInfo(-O2 -g -DNDEBUG)로 설정하고, 명시 Debug/Release와 multi-config 선택은 보존했다. fast-math나 안전 판정 완화는 없다. Optimized navigation132tests가 통과했다.

이전 시간 회귀 실패 이후 boot ID가 변경됐으며 /tmp 로그 일부가 사라졌다. 따라서 과거 실패와 현재 통과의 원인을 compiler optimization 하나로 단정하지 않는다. 이전 colcon XML은 영구 artifact로 보존했다. 현재 부팅에서 같은 source/입력/라이브러리를 사용해 O0(-O0 -g -DNDEBUG) 별도 build와 O2를 순차5회 비교했다. fixed-heading corridor rejection 중앙값116→18ms(6.44배), saved-return sweep 회귀 전체 중앙값28→30ms로 후자는 개선을 입증하지 못했다. 두 모드 모두 검색 예산/지도 접근/정상 종료 회귀를 통과했다.

실행 결과는 static fixture의 계산/거부 결과이며 Gazebo 주행 CPU/RTF/왕복 성공의 증거가 아니다. 최적화 후 시뮬레이션은 실행하지 않았다. 현재 부팅의 domain224/231 integration runtime 읽기 전용 감사는 remaining=[]였다. artifact는 optimized-build-20261004-170804/에 있다. 전체 미션 개선 완료는 계속 미충족이다.


## 2026-10-04 pan 관찰과 주행 분리

Gazebo의 `safety_mode=reactive`는 pan 정렬·새 depth·voxel 판정을 주행 허가에서
분리한다. Pan 노드는 비동기 관찰만 하고, 기존 Collision Monitor 뒤의 직접
LiDAR/depth planar 제동 guard가 유일한 최종 속도 publisher다. 30°→25°는
관찰 목표를 유지한다. Costmap/관측 memory, 속도 상한·제동값·충돌 구역·freshness·
도착 오차·미션 timeout은 유지했다. 실제 로봇 기본 모드는 기존 serial이다.

같은 GUI 조건 각 1회에서 43/13 왕복 wall 시간은 31.98/110.65→21.41/89.30초,
13번 복귀의 GT 정지 수준 시간은 9.57→2.15초, 0.2초 이상 episode는 15→2회였다.
모든 왕복이 Nav2/mission SUCCESS였지만 실측 RTF는 1.43–1.45로 **2배속 미달**이다.
13번 홈 GT 오차 12.32cm도 남았다. 창을 줄인 추가 실행도 1.45–1.47배였고,
기존 SlowZone 감속과 DWB 가속 명령의 상호작용으로 약 0.01m/s creep가 관측됐다.
Pan 정지 제거가 모든 저속 변동을 해결했다는 결과는 아니다.

Navigation 155개 테스트와 ROS 장애/복구 27개 경우를 통과했다. 마지막 cmd/GT twist
0, runtime IDLE을 확인하고 Gazebo·ROS·backend·GUI를 종료했다. 전체 사각지대 및
실물 제동은 미검증이다. 세부 비교, 제외한 prototype, 속도 그래프와 종료 근거는
`artifacts/navigation-reactive-20261004/RESULTS.md`를 따른다.
