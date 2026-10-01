# 새 CAD 모델 전체 수거 실행 기록 — 2026-09-09

> 날짜별 시험 당시의 코드·물체·설정 기록이다. 지갑, 외부 선반, 예전 깊이·속도·
> 검증 기본값을 현재 실행 설정으로 적용하지 않는다. 현재 설정은
> [Skill Executor README](../ros2_ws/src/cleany_skill_executor/README.md)와
> [MuJoCo README](../ros2_ws/src/cleany_mujoco_sim/README.md), 현재 검증 상태는
> [2026-09-14 품질 점검](QUALITY_REVIEW_20260914.md)을 따른다.

## 범위와 판정

- Ubuntu ARM64 VM, native ROS 2 Humble, MuJoCo headless. 실물 로봇은 구동하지 않았다.
- Gemini `gemini-3.1-flash-lite` 인식, SAM2 tracking 활성화, 기존 속도·충돌·파지 확인 기준 유지.
- 대상: 현재 작업 책상의 종이컵, 레고, 휴지뭉치, 지갑 4개. 다른 책상으로의 이동 작업은 범위 밖이다.
- 전체 모드 실행 후 첫 물체 실패로 중단되면, `sorting_test_only_label`로 나머지 대상을 독립 시험한다.
  독립 시험은 매번 장면을 초기화하며 연속 전체 수거 성공으로 집계하지 않는다.
- 성공 기준은 검증된 수거함 배치와 작업영역 비움 확인이다. 인식, 접촉, launch 종료 코드 0은 성공 기준이 아니다.
- 접촉/물체 위치 GT는 **사후 진단용** rosbag에만 기록하며 인식·경로 계획에 공급하지 않는다.

## 실행 결과

### 2026-09-11: 파지 후 Depth 최신성 대기 수정

- 원래 GUI 실행에서는 컵 접촉 유지 확인 후 attachment barrier에서 실패했다.
  `cloud_age=4.596s`, `map_receipt_age=0.631s`, 허용 2초, 대기 상한 10초였다.
- 손목 모드의 접근·닫기·lift 구간에만 `head_depth_boost=true`를 요청한다.
  선택 손목/SAM2는 유지하고 헤드 목표 빈도를 idle 2Hz에서 active 10Hz로 올린다.
  정상 종료와 예외 시 해제한다. Depth 입력은 KEEP_LAST(1)로 제한한다.
  attachment 이후 촬영 조건, 2초 최신성, 10초 대기 상한은 완화하지 않았다.
- `make test-grasp-pregrasp`: 896개 통과. 추가 서비스 거부/손목 선택 보존 테스트 및
  barrier 진단 로그 추가 후 관련 Python 169개 통과.
- 컵 단독 headless/SAM2 활성 실행: `/tmp/cleany-depth-boost-cup.log`,
  artifacts `artifacts/depth_boost_cup_20260911/`. 접근 궤적 `GOAL_TOLERANCE_VIOLATED`로
  파지 전에 실패. boost 해제 확인. Depth barrier 검증까지 도달하지 않았다.
- 연속 headless/SAM2 활성 실행: `artifacts/depth_boost_sequence_20260911/pipeline.log`.
  레고 attachment 이후 Depth 대기 1.569초, 새 촬영 시각은 attachment 이후 0.214초.
  lift/수거/복귀 및 독립 배치 검증 성공(center [-0.1116, 0.1011, 0.2598]).
  컵은 접근 중 right arm joint 1 위치 오차 -0.102983rad(허용 0.01rad),
  goal_time 초과 2.003925초로 실패. 약 202.522초, 수거 1/4.
- 레고에서 최신 Depth barrier 통과와 정상 boost 복원을 검증했으나, 컵의 파지 후
  lift 성공이나 원래 GUI 부하에서의 해결까지 검증한 것은 아니다. 접근 실패는 별도 잔여 문제다.

| 실행 | 대상 | 결과 | 확인된 중단 원인 |
|---|---|---|---|
| 1 | 전체, 첫 대상 레고 | 수거 0개 | 닫힘 -0.29996rad / 목표 -0.30000rad, 잔차 0.00004rad. 파지 확인 실패 |
| 2 | 컵 독립 시험 | 수거 0개 | 접촉 정지 후 다음 경로의 OctoMap–left_gripper_frame 충돌 |
| 3 | 휴지뭉치 독립 시험 | 수거 0개, 들어올리기 확인 | `No transport IK for trash_right` |
| 4 | 지갑 독립 시험 | 수거 0개, 닫힘 전 중단 | 접근 경로의 OctoMap–left_gripper_frame 충돌 |
| 5 | 레고 독립 재시험 | 수거 0개 | 파지 확인 실패 재현, 기록상 손가락–레고 접촉 없음 |

실행 1의 coordinator 시작→실패는 약 67.5초다.
실행 2는 coordinator 시작→실패 약 60.5초, 빌드·종료 포함 약 66.9초다.
실행 3은 각각 약 89.3초 / 98.2초, 실행 4는 약 41.0초 / 47.3초다.
실행 5는 빌드·종료 포함 약 68.5초다. 5회 모두 중단됐으며 검증된 수거함 배치는 **0/4개**다.
설정 변경, 충돌 무시, 파지 확인 우회 없이 실행했다. 종료 후 시뮬레이터와 recorder가 남지 않은 것을 확인했다.

### 레고 재시험: 접촉 기록

- 닫힘 실제 -0.30001rad / 목표 -0.30000rad, 잔차 -0.00001rad.
- 기록된 접촉에서 레고는 책상과의 접촉만 확인됐으며 고정/이동 손가락과의 접촉은 없었다.
  접촉 개수 상한 초과 경고는 없었다. 최대 10Hz 시뮬레이션 시간 표본이므로 짧은 순간 접촉까지 배제하지는 않는다.
- 물체 중심은 약 3.6mm 수평 이동했고 높이는 약 0.34570m로 유지됐다. 지속 파지나 들어올리기 증거가 없다.
- 접근 깊이/횡방향 오프셋/작은 물체의 추정 형상과 실제 jaw 접촉 영역을 우선 대조할 필요가 있다.

### 휴지뭉치: 파지/들어올리기 후 수거함 IK 실패

- 닫힘 실제 0.16562rad, 목표 -0.30000rad, 잔차 0.46562rad.
- 사후 접촉 기록에서 양쪽 손가락과 휴지뭉치 접촉을 확인했다.
- 물체 충돌 형상 중심의 base_link 높이는 0.36244m → 0.51567m로 약 15.3cm 상승했다.
- `transport`에 진입했지만 수거함 위 목표에 대한 현재 IK가 해를 반환하지 않았다.
  이 실패는 모든 자세/모든 팔로 절대 도달 불가능하다는 증거는 아니다.
- 코드상 선택한 팔을 그대로 사용하며, 물체 중심 보정 반복 중 IK가 None이면 즉시 중단한다.
- 종료 중 MoveIt 외에 `ros2_control_node`에서도 exit -11이 기록됐다. 작업 오류 이후 종료 오류다.

### 지갑: 닫힘 전 접근 경로 충돌

- 오류: `seeded cubic collision/constraint failure at 159/240: <octomap>/left_gripper_frame
  position=(0.4800,0.1337,0.4397) frame=base_link depth=0.000321m; constraints=[]`.
- 컵과 같은 링크/OctoMap 쌍이지만 지갑은 닫힘 명령 전 실패다. 두 실행의 원인이 같다고 단정하지 않는다.

### 컵 접촉 및 경로 실패 근거

- 그리퍼 실제 0.82695rad / 닫힘 목표 -0.30000rad, 잔차 1.12695rad.
- 접촉 기록에서 이동 손가락과 컵 벽, 고정 손가락과 컵 벽의 접촉이 모두 확인됐다.
- 기록된 개별 접촉점의 최대 법선 힘은 약 22.89N이다. 물체 전체 파지력이나 실물 힘 추정값으로 해석하지 않는다.
- 컵 충돌 형상 bounding-box 중심 높이는 처음 0.38751m, 마지막 0.38878m(base_link 기준).
  들어올리기나 수거함 배치 성공을 뒷받침하지 않는다.
- 경로 오류: `seeded cubic collision/constraint failure at 23/181: <octomap>/left_gripper_frame
  position=(0.4100,0.1400,0.5000) frame=base_link depth=0.000037m; constraints=[]`.
- 이 메시지만으로 실제 장애물/잔여 물체 포인트/자기 형상 필터 누락 중 원인을 확정할 수 없다.
  작은 침투값이라는 이유로 충돌 검사를 비활성화하지 않았다.

## 재현 자료

- 전체 실행: `artifacts/all_objects_20260909_attempt1/pipeline.log`, `run-*/stages.jsonl`, 단계별 `.cdr`.
- 독립 실행: `artifacts/all_objects_20260909_diagnostics/attempt*/` 아래 `pipeline.log`, `result.json`,
  `run-*/stages.jsonl`, 단계별 `.cdr`, 서비스 요청/응답 기록.
- 독립 실행 rosbag: `/simulation/contact_diagnostics`, `/simulation/sorting_ground_truth`, `/joint_states`, `/clock`.
- `contact_summary.json`은 rosbag에서 추출한 접촉 및 물체 위치 요약이다.
- `artifacts/`는 로컬 생성물로 Git에 포함되지 않는다. 커밋되는 이 문서와 별도로 보관해야 한다.
- 진단 실행 도구 `run_trials.py`는 각 실행에 300초 한도를 둔다. API key는 사용자 설정 파일에서 읽고 로그에 기록하지 않는다.
- 이번 실행에서 시간 한도에 걸린 독립 시험은 없다. 로그·bag·CDR 합계는 약 49MB다.

## 주의할 공통 오류

- 실패 후 launch가 0으로 종료될 수 있으므로 반드시 `SORTING failed` 및 `completed`를 확인한다.
- 종료 과정의 MoveIt segmentation fault는 최초 수거 실패와 분리해서 분석해야 한다.
- CPU SAM2 지연 및 선택적 후처리 확장 로드 경고는 별도 기록한다. 경고만으로 파지 실패 원인이라고 단정하지 않는다.

## 다음 수정 우선순위

1. 들기 전에 후면 수거함까지의 IK/이동 가능성을 사전 검증하고, 실패 원인이 위치·관절 한계·물체 오프셋 중 어디인지 분리한다.
2. 컵/지갑의 OctoMap 충돌 지점에 대해 센서 포인트, 로봇 자기 형상 마스크, 파지 물체 제거 범위를 대조한다.
3. 레고의 실제 jaw–물체 정렬과 접근 깊이를 검증한다. 판정 임계값만 낮춰 성공 처리하지 않는다.
4. 종료 중 MoveIt/ros2_control_node crash 및 실패 시 launch 종료 코드 전달을 별도로 개선한다.

이번 요청에서는 실행과 증거 수집을 수행했으며 위 제어/복구 동작 수정은 아직 구현하지 않았다.

## 후속 GUI 크기 비교

다른 파지·마찰·질량 설정은 유지하고 레고 시각/충돌 형상만 확대했다.

| 원래 대비 크기 | GUI 실행 결과 |
|---|---|
| 1.5배 | 닫힘 실제 -0.29997rad, 잔차 0.00003rad. 파지 확인 실패 |
| 2배 | 닫힘 실제 -0.03231rad, 잔차 0.26769rad. 파지 확인 및 들어올리기 동작 후 transport 진입, `No transport IK for lost_items_left`로 실패 |

2배 실행의 coordinator 시작→실패는 약 110.6초다. 수거함 배치 완료는 0개다.
이는 이번 2배 실행에서 파지 확인을 통과했다는 결과이며 반복 성공률이나 독립 접촉 GT 검증을 뜻하지 않는다.
크기 확대만으로 후면 수거함 IK 문제까지 해결되지는 않았다.
로그 및 단계별 자료: `artifacts/lego_1p5_gui_20260910/`, `artifacts/lego_2x_gui_20260910/`.

## 공통 경유점 적용 및 GUI 재시험

- 공통 TCP 목표는 base_link `[-0.35, 0, 0.60]`m. 수거함 중심 x=-0.405m보다 로봇 쪽 5.5cm이며 y는 두 함의 중간이다.
- 파지 전 빈 팔 endpoint IK/충돌/FK를 확인하고, 파지 후에는 부착 물체를 포함해 재계산 후 기존 collision-aware 이동을 사용한다.
- 전체 경로 사전 검증이나 검증된 고정 관절 자세 재생은 아니다. 최종 bin IK는 여전히 경유점 도착 뒤 수행한다.
- GUI 2회 모두 오른팔의 사전 경유점 검사와 2배 레고 파지 확인을 통과했다.
- 그러나 2회 모두 들기 경로의 sample 94/166에서 부착 물체와 OctoMap 충돌로 중단됐다.
  첫 충돌 깊이는 0.000037m, 재시험은 0.000028m. 대략 base_link `(0.376,-0.190,0.450)`m 지점이다.
- coordinator 시작→실패는 각각 약 86.3초 / 79.4초. 실제 경유점 이동에는 도달하지 않았고 수거는 0개다.
- 기존 2배 GUI 실행에서 들기가 통과한 적이 있으나 이번 두 실행에서는 실패했다. 경유점 endpoint 검사와 들기 경로 검사는 구분한다.
- 오류를 우회하지 않았으며 로그는 `artifacts/common_waypoint_20260910/pipeline-attempt1.log`,
  `artifacts/common_waypoint_retry_20260910/pipeline.log`와 각 디렉터리의 단계/서비스 기록에 저장했다.
- `make test-grasp-pregrasp` 통과 후 사전 검사 실패 시 이동 방지 테스트를 추가해 coordinator 테스트 112개를 다시 통과했다.

## OctoMap 자기 형상 누락 수정

- migrated URDF의 양팔 Fixed_Jaw_Motor와 손목 카메라 2개 메시가 visual-only여서,
  collision geometry 기반 자기 필터가 해당 표면을 제거하지 못했다.
- 마지막 실패의 실제 이동 시작 관절 상태를 재구성하면 문제 좌표와 오른쪽 camera1 메시 정점 간 최단 거리는 약 9.35mm였다.
  이는 원본 OctoMap 포인트의 직접 출처 증명은 아니지만 자기 형상 누락과 부합한다.
- 양팔의 세 메시 각각에 visual과 동일한 origin/scale의 URDF collision을 복원했다.
  MuJoCo 물리 접촉, 마찰, 이동 속도, OctoMap padding, collision 허용 기준은 바꾸지 않았다.
- 재발 방지 테스트는 모터 및 카메라의 visual/collision 양쪽 pose/scale을 확인한다.
- 수정 후 GUI 실행: 2배 레고 파지 → 들어올리기 → 공통 경유점 이동 **성공**.
  `MoveIt execution succeeded: transport via common waypoint`가 기록됐다.
- 기존 sample 94/166의 OctoMap 충돌은 이번 실행에서 재현되지 않았다.
- 이후 최종 분실물함 IK는 여전히 `No transport IK for lost_items_left`로 실패했다.
  coordinator 시작→최종 실패 약 143.8초, 수거함 배치 0개. 이 IK 문제는 별도 미해결이다.
- 빌드 및 `make test-grasp-pregrasp` 통과. 로그: `artifacts/self_filter_fix_20260910/pipeline.log`.

## 최종 수거함 영역 탐색 수정 및 레고 배치 성공

기존 방식은 수거함 중앙/고정 높이 한 점에 대해 현재 TCP 회전으로 오프셋을
보정하고 position IK를 호출했다. 저장된 오른팔 레고 파지 오프셋으로 수치적으로
비교하면 `(-0.405, 0.105, 0.4)`m 물체 중심은 최소 오차 약 6.1cm였으나,
로봇 쪽 입구 내부 `(-0.37, 0.075, 0.5)`m에는 위치 해가 있었다.
이 수치 비교 자체는 충돌/경로 검증이 아니므로 runtime MoveIt 검사를 유지했다.

- 수거함 위치·크기를 유지하고, 벽/물체 bounding sphere/5mm 여유를 뺀 입구 내부의
  물체 중심 영역을 탐색한다. 물체 아래면과 테두리 사이 수직 여유는 6–21cm다.
- runtime URDF의 `TCP 위치 + TCP 회전 × 실제 파지 오프셋`을 직접 최적화한다.
  다중 seed/반복 횟수는 제한하고, MoveIt FK 및 부착 물체 충돌을 독립 검사한다.
- 실제 이동/파지 유지 감시, 놓기 직전 실제 위치 검사, 사후 배치 검증은 유지했다.
- 첫 GUI 실행은 Gemini가 레고를 review로 보내 컵을 선택했다. 컵은 경유점까지
  도달했으나 `Payload bounding sphere does not fit inside bin opening`으로 중단됐다.
  원통/OBB 방향을 고려하면 가능한 컵을 구형 근사가 과도하게 제한하는 문제는 남아 있다.
- 이어 레고만 선택하는 독립 GUI 시험을 수행했다. **파지 → 들기 → 공통 경유점 →
  분실물함 위 이동 → 놓기 → 복귀 → VerifyPlacement 성공 → complete**를 확인했다.
- 레고의 실제 release-region bounds는 base_link
  `(-0.444317,0.070683,0.397683)..(-0.365683,0.139317,0.547683)`m였다.
- coordinator 시작→레고 complete 약 **172.1초**. 이후 다른 물체 3개가 남아
  `Work area is not clear: 3 unresolved objects`로 종료된 것은 단일 대상 진단 모드의
  정상적인 전체 비움 실패다. 레고 완료는 1개이며 전체 4개 수거 성공은 아니다.
- 로그: `artifacts/release_region_20260910/pipeline.log`(컵),
  `artifacts/release_region_lego_20260910/pipeline.log`(레고 성공), 각 run의 stage/CDR.
- GUI 및 관련 실행 프로세스는 종료됐다. 컵의 방향별 입구 크기 검사와 다른 물체의
  최종 배치는 별도로 검증/개선해야 한다.

## MuJoCo 놓기/공중 부유 재현 진단

사용자가 MuJoCo에서 레고가 열린 그리퍼 사이에 떠 있다가 복귀 중 떨어졌다고
보고해, 접촉·물체 위치·관절·clock·sorting status를 rosbag에 기록하며 GUI 재시험했다.
이번 요청에서는 실행 설정과 진단만 수행했고 제어/물리 코드는 변경하지 않았다.

1. 목표 배속 2.0, RViz 없음: 초반 현실 33.35초/시뮬레이션 10.245초(RTF 0.307).
   경유점 이동 중 지속 접촉 상실로 중단돼 release에 도달하지 못했다.
   실행 중 DB 조회와 기록이 겹친 뒤 SQLite lock 오류로 recorder가 종료됐다.
   첫 bag은 불완전하며 놓기 원인 분석에 사용하지 않는다.
2. 목표 배속 1.0, RViz 없음: Gemini가 레고를 review로 분류해 파지 전 중단.
3. 목표 배속 1.0, MuJoCo+RViz: 레고 분실물함 배치 검증 성공. 기록 종료 후 분석했다.
   전체 clock 기준 RTF 약 0.2314. 레고 완료 후 나머지 3개 unresolved로 종료됐다.

세 번째 실행의 release 접촉/높이 기록(base_link, 높이는 충돌 AABB 중심):

| 시뮬레이션 시간 | 레고 높이 | 확인된 접촉 |
|---|---|---|
| 35.169s | 0.5461m | 양쪽 손가락 |
| 35.271s | 0.5255m | 없음 |
| 35.391s | 0.4103m | 없음 |
| 35.522s | 0.2210m | 분실물함 |
| 37.231s | 약 0.2198m | 그리퍼 완전 열림 시점에는 이미 함 안 |
| 37.508s | 약 0.2198m | 복귀 시작 시점에도 함 안 |

마지막 손가락 접촉 표본→첫 수거함 접촉 표본은 시뮬레이션 0.353초,
현실 약 1.31초였다. 완전 열림 이후 복귀까지의 현실 약 1.02초는
시뮬레이션 약 0.277초에 해당했다. **이번에는 공중 부유가 재현되지 않았다.**
이 결과로 사용자가 본 이전 실행의 원인을 확정하거나 부정하지 않는다.
목표 배속만 높여도 CPU/SAM2/렌더링 처리 한계를 넘을 수 없으며,
2.0 목표 실행의 접촉 실패를 배속 때문이라고 단정할 근거도 없다.

자료: `artifacts/lego_release_diagnostics_20260910/`(불완전 첫 기록),
`artifacts/lego_release_diagnostics_retry_20260910/`(review),
`artifacts/lego_release_diagnostics_third_20260910/`(완전한 bag, pipeline.log,
`release_timeline.json`, 읽기 전용 분석 스크립트). 최종 분석 뒤 GUI/recorder 모두 종료했다.

## 가까운 순 전체 수거 headless 실행

사용자 요청으로 `sorting_test_only_label`을 지정하지 않고, Gemini + SAM2 tracking,
공통 경유점/영역 IK를 유지한 채 GUI 없이 전체 모드를 실행했다.

- 첫 대상 레고: 거리 0.55528m, 분실물함 배치/복귀/검증 성공. 시작 후 약 163.9초에 complete.
- 재관측 후 컵: 거리 0.60164m, 왼팔 파지/들기/공통 경유점 도착 성공.
- 컵의 최종 투하 영역 계산에서 `Payload bounding sphere does not fit inside bin opening`으로 중단.
  구형 근사가 컵의 크기를 보수적으로 제한하는 기존 미해결 문제이며, 이 실행에서는 기준을 우회하거나 수정하지 않았다.
- 총 coordinator 실행 약 238.6초(3분 59초), 검증된 수거 **1/4개**.
- 컵은 수거 미완료, 휴지뭉치/지갑은 해당 연속 실행에서 아직 시도하지 못했다.
- 전체 작업 비움 확인은 실패했다. 단일 대상 독립 시험과 달리 이번 것은 실제 연속 실행 결과다.
- 로그/단계/CDR: `artifacts/nearest_all_headless_20260910/`. 모든 실행 프로세스는 종료됐다.

## 컵 80% 축소 후 가까운 순 전체 수거 재시험

- GUI 없이 Gemini Flash-Lite + SAM2 tracking, 전체 대상 선택으로 실행했다.
- 레고(0.55585m): 파지부터 분실물함 배치/복귀/검증까지 성공. 시작 후 약 172.4초에 complete.
- 다음 컵(0.59308m): 왼팔 파지/들기/공통 경유점 이동 성공.
- 축소 컵은 이전의 수거함 입구 크기 검사를 통과했지만, 최종 영역 IK가
  `No collision-free release region IK for trash_right`로 실패했다.
  검색 영역은 base_link `(-0.421113,-0.116113,0.420887)..(-0.388887,-0.093887,0.570887)`m.
  이는 현재 탐색에서 유효한 자세를 찾지 못했다는 의미이며, 모든 가능한 자세가
  물리적으로 불가능하다는 증거는 아니다. 이번 실행에서는 제어 코드를 수정하지 않았다.
- 총 coordinator 실행 약 241.4초(4분 1초), 검증된 수거 **1/4개**.
  컵 수거는 미완료이고 휴지뭉치/지갑은 시도 전 중단됐다.
- 로그/단계/CDR: `artifacts/nearest_all_cup80_20260910/`. 실행 프로세스 종료 확인.

## 컵 80% 설정 GUI 전체 수거 재시험

- 동일 설정에서 MuJoCo와 RViz를 켜고 전체 가까운 순 모드를 실행했다.
- 첫 레고(0.55596m)는 오른팔 파지/들기/공통 경유점/분실물함 이동/
  그리퍼 열기/복귀까지 실행 성공했다.
- 이후 배치 검증이 10초 뒤 `Placement could not be verified in destination`으로 실패했다.
  따라서 검증된 수거는 **0/4개**이고 컵/휴지뭉치/지갑은 시도하지 못했다.
  로그만으로 실제 낙하 위치나 검증 실패의 구체적 원인은 확정하지 않는다.
- coordinator 시작부터 실패까지 약 205.2초(3분 25초).
- 로그/단계/CDR: `artifacts/nearest_all_cup80_gui_20260910/`.
  실패 후 launch가 GUI와 관련 프로세스를 자동 종료했다. 코드 변경은 없다.

## 레고 높이 추가 10% 확대 GUI 재시험

- 레고 총 높이 25.08mm(몸체 21.12mm), 가로/세로/질량/마찰 유지 상태로
  MuJoCo + RViz 전체 가까운 순 수거 모드를 실행했다. Gemini Flash-Lite와 SAM2 tracking 유지.
- 첫 레고(0.55626m)의 파지/들기/공통 경유점/분실물함 이동/놓기/복귀 실행은 성공했다.
- 배치 검증에서 10초 뒤 `Placement could not be verified in destination`으로 중단됐다.
  높이 증가만으로 직전 실패가 해결되지 않았다. 실제 낙하 위치와 구체적 실패 원인은
  이 실행 로그만으로 확정하지 않는다.
- coordinator 시작부터 실패까지 약 193.1초(3분 13초). 검증된 수거 **0/4개**,
  컵/휴지뭉치/지갑은 미시도. 실패 처리로 GUI와 관련 실행 프로세스가 종료됐다.
- 로그/단계/CDR: `artifacts/nearest_all_lego_taller_gui_20260910/`.

## 파지 후 손목 관절 유지 구현 검증

- 안정된 접촉 시점의 pitch/roll 실제 관절각을 캡처하고, 놓기 후 제한을 해제한다.
  IK의 파지각 기준 허용 범위를 2/5/10/20/30도로 제한하며 경로 제약 및 직접
  Cartesian waypoint 검사에도 적용했다. 최대 범위 초과는 중단한다.
- 관련 coordinator/adapter 테스트 169개 통과, `make build-grasp-pregrasp` 9개 패키지 빌드 성공.
- 추가 전체 skill executor 테스트: 494 passed, 1 skipped, 2 failed.
  `test_selected_grasp_is_executed_in_mujoco` 런타임 데모 실패가 있었고,
  이후 `test_nearest_failure_falls_back_and_executes_pregrasp` 대기 중 테스트 프로세스에
  SIGINT를 보내 종료하면서 rclpy 중복 shutdown 실패가 발생했다.
  따라서 전체 런타임 회귀 통과로 간주하지 않는다. 테스트의 launch 자식 프로세스 종료 확인.
- 이 변경으로 실제 수거 성공이나 손목 회전 감소량을 확인한 것은 아니며,
  변경된 전체 수거 파이프라인의 GUI/물리 검증은 아직 수행하지 않았다.

## 손목 제한 적용 후 전체 수거 headless 시험

- Gemini Flash-Lite + SAM2 tracking, 대상 제한 없이 가까운 순으로 실행했다.
- 레고(0.55637m) pregrasp/접근/접촉 파지 확인 이후 `reverse grasp retreat`
  계획이 `seeded cubic collision/constraint failure at 11/167`로 실행 전 거부됐다.
  로그에 접촉 물체 쌍은 없고 실패 constraint distance는 0.038923335였다.
- 저장된 motion request에서 손목 pitch 기준 0.663603rad, roll 기준 -2.738803rad,
  두 관절 허용 범위 ±0.034907rad(2도)를 확인했다. 기존 역방향 후퇴 궤적과
  추가 손목 제약의 불일치가 유력하지만, 로그의 constraint distance만으로
  실패 관절이나 각도를 확정하지 않는다.
- 현재 자동 완화는 IK 탐색에만 적용되므로 직접 궤적의 제약 실패에서는
  허용 범위를 넓혀 재계획하지 않고 중단하는 한계가 실제 파이프라인에서 드러났다.
- coordinator 시작부터 실패까지 63.0초. 검증된 수거 **0/4개**,
  컵/휴지뭉치/지갑은 미시도. 실행 프로세스 종료 확인.
- 로그/단계/CDR: `artifacts/wrist_hold_pipeline_20260910/`. 이번에는 코드 수정 없이 시험했다.

## 손목 제한 동일 조건 재현 시험

- 코드/설정을 변경하지 않고 동일 headless 전체 수거를 재실행했다.
- 첫 레고(0.55626m) 접근 및 접촉 파지 이후 다시
  `seeded cubic collision/constraint failure at 11/167`로 중단됐다.
- 실패 constraint distance는 0.038923903204082566으로 직전 0.03892333511759516과
  거의 같다. 같은 경로 샘플에서 같은 종류의 제약 실패가 2회 연속 재현됐다.
- coordinator 실행 64.0초, 검증된 수거 0/4개. 나머지 3개는 미시도.
- 로그/단계/CDR: `artifacts/wrist_hold_retry_20260910/`. 실행 프로세스 종료 확인.

## 손목 제한 후퇴 경로 수정 및 전체 파이프라인 검증

저장된 실패 request를 디코딩한 결과, 기존 후퇴 endpoint는 파지 상태 대비
오른쪽 wrist pitch **38.8932도**, roll 0.4883도 변화를 요구했다. 2도 제한과
양립하지 않는 빈 팔 pregrasp endpoint를 그대로 사용하는 것이 원인이었다.
local Cartesian refinement도 손목 경계를 생성 단계에 반영하지 않고 사후 검사만 했다.

수정 내용:

- 후퇴/lift endpoint를 파지 기준 손목 경계 안에서 다시 계산한다.
- local refinement의 관절 경계를 물리 한계와 path constraint의 교집합으로 제한한다.
- 계획/검증을 실행에서 분리하고, `CartesianPlanningError`에만 다음 허용 범위로
  재계획한다. 서비스/URDF 오류 및 제어/접촉 실패 후에는 재실행하지 않는다.
- 손목을 유지하며 다른 관절을 움직일 때 변하는 TCP 세계좌표 방향을 새로 계산한다.
  구간 시작 대비 최대 30도(설정 가능), 전체 FK 방향 보간 오차/충돌/속도 검사는 유지한다.

첫 수정 시험(`artifacts/wrist_replan_20260910/`)은 새 IK 해가 기존 pregrasp
세계좌표 방향과 달라 실패했다. 이후 위의 별도 carry 방향 조건을 적용해 다시 실행했다.

최종 전체 headless 실행(`artifacts/wrist_replan_v2_20260910/`):

- 레고: 후퇴 경로 및 실행 통과, 공통 경유점 통과, 분실물함 배치/복귀/검증 성공.
  coordinator 시작 후 152.9초에 완료. 후퇴/경유점은 손목 ±2도, 최종 함 IK만 ±20도 사용.
- 컵: 후퇴와 경유점 통과. 쓰레기함 영역 IK는 ±2/5/10/20/30도 모두 실패해 중단.
  남은 오류는 `No carry IK within configured grasp-relative wrist bounds`이다.
- 총 223.0초, 검증된 수거 **1/4개**. 휴지뭉치/지갑은 미시도.
- 실행 전 검증된 후퇴 trajectory의 파지각 대비 최대 편차는 레고 pitch 2.0도,
  roll 약 0.0000072도, 컵 pitch 2.0도, roll 약 0.0000056도였다.
  이 수치는 **계획 궤적** 기준이며 연속 실측 관절 기록으로 계산한 값은 아니다.
- 이전 `11/167` 오류는 두 물체에서 재현되지 않았다. 전체 4개 수거 성공은 아니다.
  프로세스 종료 확인. 로그/서비스 CDR/계획 trajectory를 보존했다.
- 최종 `make test-grasp-pregrasp` 성공: Python 823개 + C++ 44개, 총 **867개 통과**.
  9개 관련 패키지 빌드 성공. 별도의 이전 런타임 데모 전체가 통과했다는 뜻은 아니다.

## 수정본 동일 조건 전체 수거 재시험

- 코드/설정 변경 없이 Gemini Flash-Lite + SAM2 tracking, headless 가까운 순 수거 실행.
- 레고: 후퇴 ±2도, 경유점 ±2도, 최종 분실물함 IK ±20도로 수거/복귀/검증 성공.
  시작 후 151.7초에 완료했다.
- 컵: 왼팔 파지 및 후퇴 ±2도, 공통 경유점 이동까지 성공했다.
  그 뒤 쓰레기함 투하 영역 IK가 ±2/5/10/20/30도에서 모두 실패했다.
  오류는 직전과 같은 `No carry IK within configured grasp-relative wrist bounds`.
- 실패 위치는 **공통 경유점 도착 후 최종 쓰레기함 endpoint 자세 탐색**이다.
  최종 함 이동/그리퍼 열기는 시작하지 않았으며, 후퇴 샘플 `11/167` 오류는 재발하지 않았다.
  이 로그만으로 충돌 물체 쌍이나 손목 제한 단독 원인을 확정하지 않는다.
- 총 coordinator 시간 224.6초(3분 45초), 검증된 수거 1/4개. 휴지뭉치/지갑은 미시도.
- 자료: `artifacts/wrist_replan_retest_20260910/`. 실행 프로세스 종료 확인.

## 고정 투하점 / 경유점 이후 손목 고정 구현

사용자 요청으로 기본 수거 모드를 고정 물체 중심 투하점으로 변경했다.
경유점 이전에 투하점과 경유점 양쪽 endpoint의 공통 손목 자세를 검사한다.
투하점의 첫 IK 후보가 경유점에 도달하지 못하면 다음 IK 후보를 계속 검사한다.
경유점 뒤에서는 실제 손목 feedback을 캡처해 IK의 두 변수를 완전히 고정하고
나머지 세 관절만 푼다. 최종 경로 tracking 오차는 0.5도이며 자동 확대하지 않는다.
캐시의 정확한 팔/손목/offset/투하점 일치 여부와 현재 FK/충돌을 검사한 후 재사용한다.
함 입구/물체 중심/손목 feedback을 확인한 뒤 그리퍼를 여는 조건도 유지한다.

- 첫 시험: 분실물 투하점 `[-0.395,0.105,0.52]`m. 레고 파지/후퇴 이후,
  공동 손목 endpoint 해를 찾지 못해 경유점 출발 전에 실패. 약 85.5초, 수거 0/4개.
  자료: `artifacts/fixed_release_20260910/`.
- 두 번째 시험: 분실물 투하점을 `[-0.375,0.085,0.52]`m로 조정하고 경유점과
  연결 가능한 다음 IK 후보까지 검사하도록 보완했다. 레고 파지/후퇴 이후
  동일하게 ±2/5/10/20/30도에서 공동 endpoint 해를 찾지 못했다.
  오류: `No shared-wrist waypoint/fixed release IK for right/lost_items_left`.
  약 97.0초, 수거 0/4개. 자료: `artifacts/fixed_release_v2_20260910/`.
- 쓰레기함 고정 투하점은 `[-0.395,-0.105,0.52]`m. 두 전체 실행 모두 레고에서
  중단돼 컵/휴지뭉치/지갑은 아직 검증하지 못했다.
- **구현 및 단위 테스트와 실제 수거 성공은 구분한다.** 현재 고정 투하점과 파지 기준
  손목 최대 30도 조건으로 실행 가능한 조합은 아직 확보하지 못했다. IK 탐색 실패는
  수학적 불가능의 증명이 아니며, 제한을 무시하거나 무검증 자세를 강제로 실행하지 않았다.
- 실행 프로세스 종료 확인. 수거함 자체의 위치/물리 마찰은 변경하지 않았다.
- 최종 `make test-grasp-pregrasp`: Python 832개 + C++ 44개, **876개 통과**.
  빌드도 성공했다. 고정 투하의 실제 수거 성공을 의미하는 결과는 아니다.

## 2026-09-11 고정 투하 동일 설정 재시험

- 코드/설정 변경 없이 headless 전체 가까운 순 수거 실행. Gemini Flash-Lite + SAM2 tracking 유지.
- 레고 접근/파지/후퇴 통과 후, 운반 전 사전 검사에서 다시 실패했다.
  고정 투하점 `[-0.375,0.085,0.52]`m와 공통 경유점의 공동 손목 자세를
  ±2/5/10/20/30도에서 모두 찾지 못했다.
- 오류: `No shared-wrist waypoint/fixed release IK for right/lost_items_left`.
  공통 경유점 이동 및 투하 동작은 시작하지 않았다.
- coordinator 시작부터 실패까지 91.4초, 검증된 수거 0/4개. 나머지 물체는 미시도.
- 자료: `artifacts/fixed_release_retest_20260911/`. 실행 프로세스 종료 확인.

## 2026-09-11 분류된 수거함으로 직접 운반

- 기본 고정 투하 모드에서 파지 전 공통 경유점 검사, 공동 endpoint 검사,
  파지 후 경유점 이동을 제거했다. 분류→파지→안전한 후퇴/들어 올리기→
  선택된 수거함의 고정 물체 중심 좌표→놓기→복귀 흐름이다.
- 최종 이동은 들어 올린 직후 실제 손목 pitch/roll을 고정하고 나머지 세 관절로
  계산한다. 충돌/부착 물체/도착 확인은 유지하며, 수거함 좌표나 물리 조건은 변경하지 않았다.
- 전체 headless 실행: 첫 레고의 파지 및 ±2도 후퇴는 성공했다. 이후 직접 투하점 IK가
  `Direct fixed release unreachable with post-lift wrist: right/lost_items_left`로 실패했다.
  공통 경유점 stage는 실행되지 않았다. 고정 손목으로 최종 좌표에 도달 가능한 해는
  이번 탐색에서 확보하지 못했으며, 물리적 불가능의 증명은 아니다.
- coordinator 실행 86.5초, 검증된 수거 0/4개. 나머지 물체는 미시도.
- 자료: `artifacts/direct_release_20260911/`. 실행 프로세스 종료 확인.
- 최종 `make test-grasp-pregrasp`: Python 833개 + C++ 44개, **877개 통과**, 빌드 성공.

## 2026-09-11 사용자 의도 정정: 손목 roll만 유지

- 사용자는 손목 위아래 pitch 고정이 아니라 회전축 roll 유지를 요청한 것임을 명확히 했다.
  이전 pitch/roll 동시 고정은 요청을 과도하게 제한한 구현이었다.
- 파지 기준 reference를 `wrist_roll_joint` 하나만 캡처하도록 변경했다.
  이에 따라 후퇴/lift IK 경계, 경로 제약, 최종 고정 투하 IK와 feedback 검사 모두
  roll만 제한하고 pitch는 기존 물리 한계 안에서 자유롭게 사용한다.
- 공통 경유점 없는 직접 투하 흐름은 유지한다. 최종 투하 IK의 자유 관절은
  어깨 yaw/pitch, 팔꿈치 pitch, 손목 pitch의 네 개다.
- 관련 coordinator/adapter 테스트 183개 통과. 이번 수정 후 실제 전체 수거는
  아직 재실행하지 않았으므로 이전 도달 실패의 해결 여부는 미검증이다.

## 2026-09-11 roll만 유지한 전체 파이프라인 시험

- pitch 자유 / roll 유지, 공통 경유점 없이 고정 투하점 직접 이동 설정으로 headless 실행했다.
- 레고 pregrasp/파지/후퇴 통과 후 운반 단계에서
  `Direct fixed release unreachable with post-lift wrist: right/lost_items_left`로 중단됐다.
- 저장한 후퇴 motion request를 디코딩해 경로의 joint constraint에
  `right_wrist_roll_joint`만 존재하고 pitch 고정 제약은 없음을 확인했다.
- 따라서 pitch를 풀어도 현재 고정 투하점/파지 offset/roll 조건에서 IK를 찾지 못했다.
  이 시험으로 도달 범위와 충돌 및 탐색 한계 중 단독 원인을 확정하지 않는다.
- coordinator 실행 83.8초(1분 24초), 검증된 수거 0/4개. 나머지 3개는 미시도.
- 자료: `artifacts/roll_only_pipeline_20260911/`. 실행 프로세스 종료 확인.

## 2026-09-11 최종 투하 손목 제약 해제 비교 시험

- 사용자 요청으로 접근/후퇴는 기존 설정을 유지하고, 최종 운반 시작 시에만
  carry wrist reference를 비워 고정 roll IK와 MoveIt 손목 path constraint를 해제했다.
  관절 물리 한계, 현재 payload offset, 고정 물체 중심 투하점 및 충돌 검사는 유지했다.
- 레고 파지/들어 올리기는 성공했지만 최종 분실물함 고정 투하 IK는 여전히 실패했다.
  로그의 `DIAGNOSTIC: final release wrist unconstrained`가 제약 해제를 확인한다.
  실패 문자열의 `with post-lift wrist`는 기존 공통 오류 문구이며, 이번 실행에서는
  실제 fixed_joints가 빈 dict여서 손목을 고정한 시험이 아니다.
- coordinator 실행 86.9초, 검증된 수거 0/4개. 최종 운반/놓기는 시작하지 않았다.
- 이번 결과는 roll 고정 해제만으로 해결되지 않음을 보여준다. 목표점/물체 offset,
  물리 도달 범위/충돌 조건/수치 탐색 중 단독 원인을 확정하거나 불가능을 증명하지 않는다.
- 자료: `artifacts/free_wrist_test_20260911/`. 시험용 임시 코드는 제거해 기존 roll 유지
  동작으로 복구했고, 실행 프로세스 종료를 확인했다.

## 2026-09-11 고정 투하점의 기하학적 도달 거리 진단

코드 변경/모션 실행 없이 현재 URDF와 손목 제약 해제 시험의 기록을 분석했다.
selection 및 wrist handoff의 물체 중심이 동일한 base_link 좌표임을 확인했다.
후퇴 motion request의 시작 feedback을 접촉 직후 상태의 근사로 사용해 FK를 계산했다.

- 레고 중심: `(0.39979256,-0.15977353,0.35244105)`m.
- 기록 feedback의 TCP: `(0.41222678,-0.15259012,0.36793517)`m.
- 재구성한 held offset(TCP): `(-0.01100004,-0.01431148,0.01097542)`m.
- 목표 물체 중심: `(-0.375,0.085,0.52)`m.
- 오른쪽 shoulder-pitch 원점은 shoulder-yaw를 돌리면 중심
  `(0.116101,-0.185063,0.550297)`m, 반경 0.0306m인 수평 원을 그린다.
  yaw 각도 한계를 무시하고 이 원 전체에서 목표까지의 최소 거리를 계산하면
  **0.5307244m**다.
- 이후 링크 translation 길이들의 합과 마지막 TCP translation+held offset의 길이를
  합한 낙관적인 도달 상한은 **0.5253996m**다. 링크가 모두 최적으로 정렬된다는
  가정이며 관절 각도/충돌 제한은 무시한 상한이다.
- 최소 필요 거리조차 상한보다 **5.325mm** 크다. 따라서 현재 모델과 재구성된
  파지 상태 기준에서는 해당 오른팔의 고정 투하점이 기하학적 범위 밖이다.
  수치 IK 실패만 근거로 한 결론과 달리 링크 길이에 의한 별도 필요조건 검증이다.
- 접촉 직후 feedback으로 held offset을 재구성했으므로 실제 물체 위치/실로봇의
  도달 한계를 mm 단위로 확정하는 결과는 아니다. 5.3mm 이동만으로 해결된다는 뜻도
  아니다. 실제 관절 제한/충돌/여유를 포함해 로봇 쪽 투하점 또는 반대 팔을 검증해야 한다.

## 2026-09-11 내부 후면 수거함 구현 및 검증

- 기본 외부 선반을 제거하고 chassis 부착 내부 받침판(210×460×12mm)으로 변경.
  두 함 중심 base_link=(-0.075, ±0.105)m, 바닥 z=0.22m, 상단 z=0.34m.
  왼쪽 분실물/오른쪽 쓰레기 구분 및 180×170×120mm 함 크기는 유지.
- 고정 물체 중심 투하점을 (-0.075, ±0.105, 0.50)m로 함께 변경.
  MuJoCo와 MoveIt은 같은 설정에서 받침판/함의 충돌 형상을 생성한다.
- 프레임·전자장치의 chassis 박스 충돌 형상과 내부 fixture 간 관통 없음,
  자유 낙하 물체의 함 내부 정착 테스트 통과. 체결 강도/배선/CAD mesh 전체
  여유 및 실제 이동 로봇 적용은 검증 범위가 아니다.
- `make test-grasp-pregrasp`: 9패키지 빌드 성공, Python 835개 + C++ 44개 통과.
  이전 함 높이를 상수로 사용하던 placement verifier 테스트를 설정 기반으로 수정.
- headless Gemini Flash-Lite + SAM2 tracking 전체 파이프라인 시도:
  `artifacts/internal_rear_bins_20260911/pipeline.log`.
  시작→실패 약 84.3초, 완료 0개. 레고 파지/후퇴 후 transport 단계에서
  `Direct fixed release unreachable with post-lift wrist: right/lost_items_left`.
  이는 손목 roll 유지 및 충돌 검사를 포함한 IK 해 탐색 실패이며, 새 좌표 자체가
  기하학적으로 불가능하다는 증명은 아니다. 다음 검증은 손목 제한/충돌 원인 분리,
  양팔별 투하 자세 사전 탐색이다. 자동 종료 중 MoveIt exit -11도 관측됨.
- 프로세스 종료 확인. GUI는 실행하지 않음. 배치 구현 완료와 수거 성공을 구분한다.

### 내부 후면 배치 GUI 확인 후 동일 설정 재시험

- GUI 종료 후 headless, Gemini `gemini-3.1-flash-lite`, SAM2 tracking 활성화,
  sim_speed_factor=1.0으로 실행. 코드/속도/손목 제한 변경 없이 재현 여부 확인.
- `artifacts/internal_rear_bins_retest_20260911/pipeline.log`에 전체 로그 저장.
- coordinator 시작부터 실패까지 실제 87.95초: 초기 준비 11.07초,
  인식/선택 14.63초, pick 단계(접근·파지·후퇴/들기) 61.95초,
  transport 진입부터 IK 실패까지 0.30초.
- 레고를 분실물로 분류하고 오른팔 파지 후 후퇴를 수행했으나,
  `Direct fixed release unreachable with post-lift wrist: right/lost_items_left`
  재발. 수거 완료 0/4, 나머지 물체는 실행하지 못함.
- 새로운 내부 투하점에서도 유효 IK 탐색 실패가 반복됨. 로그만으로 관절 제한,
  손목 roll 고정, 충돌 제약 중 단일 원인을 확정할 수 없음.
- 종료 중 MoveIt exit -11도 다시 관측되나 최초 중단은 위 IK 실패임.
  종료 후 GUI/제어/수거/MoveIt 실행 프로세스 없음 확인.

### 분실물 왼쪽 / 쓰레기 오른쪽 배치 시험

- 사용자 요청에 따라 레고 desk offset=(-0.16,-0.205), 컵=(+0.12,-0.175)m로
  좌우를 변경. 지갑은 왼쪽, 휴지는 오른쪽 기존 위치 유지. 로봇 기준 좌우를
  실제 MuJoCo chassis 변환으로 검증하는 테스트 추가. 카메라 시야/정착 포함
  `test_study_cafe_scene.py` 17개 통과, 실행 시 9패키지 빌드 성공.
- headless Gemini Flash-Lite + SAM2 tracking, 기존 가까운 팔 선택/손목 roll
  유지/투하점 설정은 변경하지 않음. 로그: `artifacts/same_side_bins_20260911/pipeline.log`.
- 레고가 왼팔로 선택됨. 왼쪽 분실물함의 고정 투하점 IK, 운반, 그리퍼 열기 성공.
  이전 오른팔→왼쪽 함의 투하 IK 실패는 이번 레고 시험에서 재발하지 않음.
- 시작→종료 약 142.69초. 복귀 시 `left_arm_controller`의
  PATH_TOLERANCE_VIOLATED로 실패, 현재 자세에서 재계획 1회도 실패.
  재시도 순간 joint 0 위치 오차 -0.124989rad가 허용치 0.12rad 초과.
  최종 목표 대비 오차 로그와 순간 궤적 추종 오차는 구분해야 함.
- 실패 메시지 `return from lost_items_left execution failed: status=6 code=-4`.
  placement verifier 이전 복귀 단계에서 중단되어 검증 완료는 0/4;
  그리퍼 열기 성공만으로 함 내부 정착을 확정하지 않음. 다른 세 물체는 미시험.
- 프로세스 자동 종료 확인. 다음 과제는 복귀 궤적 추종 실패 원인 분리 및
  나머지 물체의 같은 쪽 수거 경로 검증. 허용 오차를 임의 확대하지 않음.

## 2026-09-11 복귀 자기 충돌 진단 및 고정 기둥 복구

- 사용자 요청으로 현재 GUI 캡처 시도. X11 root capture는 실패, GNOME screenshot은
  AccessDenied, Qt 창 캡처는 검은 화면이었다. 실제 스크린샷을 얻었다고 주장하지 않는다.
  `/joint_states` 및 접촉 기록으로 별도 MuJoCo 모델에 로봇 자세를 재구성했다.
  `artifacts/same_side_bins_gui_20260911/contact_pose_reconstructed.png`의 청색은
  기둥 충돌 형상이다. 이 그림은 로봇 관절만 재현하며 책상 물체는 초기 배치다.
- 복귀 속도/가속도 상한 0.12/0.15를 먼저 적용했으나 재현 시험에서도 실패했다.
  기존 STS3215 토크·감쇠 대비 30% yaw 속도가 과한 것은 별도 제약이나,
  이번 반복 중단의 직접 원인은 속도만이 아니었다. 추종 허용치/토크는 완화하지 않음.
- 접촉 진단 bag에서 `Lower_Arm`의 geom 66과 `top_base_link_collision_0` 사이
  최대 normal force 10.068566N, tangential force 8.385259N 관측.
  접촉 위치 base_link=(0.179600,0.060251,0.574593)m. 수거함 접촉이 아닌 자기 충돌.
- 근본 누락: MoveIt의 `include_head_camera:=false`가 고정 mast까지 제외했다.
  `cleany_head_mount`를 공통 robot geometry로 이동하여 기본/제어/MoveIt 모델 모두
  동일한 `top_base_link` 및 fixed TF를 갖도록 수정. pan/tilt는 feedback이 없는
  팔 전용 모델에서 계속 제외하며 이 범위는 별도 잔여 제약이다.
- 시도했던 lower-arm collision box 확대는 원인 해결책이 아니어서 되돌렸다.
  CAD 메시 및 MuJoCo 물리 형상은 그대로 유지. 고정 기둥 충돌 검사만 복구했다.
- 변경 후 기록된 접촉 자세에서 yaw를 기둥 쪽으로 0.01rad 더 회전한 상태는
  `/check_state_validity`가 `left_lower_arm_link`–`top_base_link` 충돌로 거부한다.
  정확한 접촉 경계는 메시 정밀도 차이로 valid일 수 있어 관통 상태로 검증했다.
- `make test-grasp-pregrasp`: Python 842 + C++ 44 = 886개 통과, 9패키지 빌드 성공.
- 복귀 분리 검증: 이전 bag의 투하 직후 시작 자세/원래 복귀 목표로 새 MoveIt 경로
  계획 후 별도 MuJoCo 모델에서 position actuator와 cubic Hermite 보간으로 재생.
  계획 시간축 길이 8.10665초, 최대 관절 오차 0.001259rad, 최종 최대 오차
  0.000389rad, lower-arm–mast 접촉 없음. 실제 ros2_control 전체 실행과는 구분한다.
  `artifacts/mast_final_20260911/isolated_return_verification.log` 및 plan CDR 참조.
- 전체 재시험은 새 인식/파지 후보에서 레고·컵·휴지가 unreachable로 분류되어
  복귀 단계까지 도달하지 못함. 복귀 분리 시험 통과를 전체 수거 성공으로 세지 않는다.
- 최종적으로 지갑도 unreachable, 약 418.04초 후 `Work area is not clear:
  4 unresolved objects`로 종료. 수거 0/4, GUI 없이 시험. 종료 시 MoveIt exit -11도
  다시 관측됨. 파지 후보/OctoMap/경로 제약을 분리하는 후속 검증과 종료 문제는 남아 있다.
  `artifacts/mast_final_20260911/pipeline.log`, `contact_bag/`에 보존.
  GUI/제어/계획/기록 프로세스는 모두 종료했다.

### 수정 후 전체 파이프라인 동일 조건 재시험

- 사용자 요청으로 코드 변경 없이 재실행. headless, Gemini `gemini-3.1-flash-lite`,
  SAM2 tracking 설정 활성화, 내부 후면 함/분실물 왼쪽·쓰레기 오른쪽 배치 유지.
  고정 기둥의 planning collision/TF와 복귀 scaling 수정 유지.
- 9패키지 빌드 성공. 시작→실패 421.90초(약 7분 2초).
  초기 준비 10.59초; 인식 및 첫 레고 후보 검사 103.04초;
  이후 컵 검사 81.50초, 휴지 검사 107.08초, 지갑 검사 119.70초.
  이는 SORTING 이벤트 사이의 실제 시간이며 순수 IK 추론 시간은 아니다.
- 네 물체 모두 인식 및 분류됨: 레고/지갑 lost_item, 컵/휴지 trash.
  각 물체의 양팔 후보 검사 후 모두 unreachable. 일부 pregrasp 경로 계획은
  성공했으나 방향 정렬 또는 최종 grasp pose-compatible IK 등에서 후보 탈락.
- pick/transport/release/retreat 실행 단계에 도달하지 못함. 수거 완료 0/4,
  최종 오류 `Work area is not clear: 4 unresolved objects`.
  실행 중 충돌/궤적 추종 실패가 아니라 동작 전 파지 후보 선택 실패의 재현이다.
  설정상 SAM2 tracking은 켜져 있지만 손목 추적 동작으로 넘어가지 않았다.
- Missing transform/PATH_TOLERANCE 로그 없음. 종료 중 MoveIt exit -11은 재발.
- `artifacts/full_retest_20260911/pipeline.log`, `contact_bag/`, run별 `stages.jsonl`
  및 서비스 요청/응답 보존. 기록기를 포함한 실행 프로세스 모두 종료 확인.

### 사용자 요청: 수거 시뮬레이션의 기둥 충돌 무시

- `robot_top_bins.yaml`에 `simulation_ignore_mast_collision: true` 추가.
  MuJoCo 임시 수거 모델에서 top_base_link 직속 geom의 접촉 비트를 0으로 설정.
  canonical MJCF, 기둥 visual, head 자식/팔/물체/함의 물리는 변경하지 않음.
- 같은 YAML을 MoveIt launch에 전달해 런타임 SRDF에서 기둥–로봇 링크 21쌍만
  충돌 제외. 원본 SRDF 및 self-filter용 collision geometry/TF는 유지한다.
  실제 시간(use_sim_time=false)에는 override를 거부하며 일반 launch는 기본 비활성.
- false로 설정하면 예외를 제거한다. 이 모드의 경로는 실제 기둥을 통과할 수 있으므로
  실제 로봇 안전성 검증 결과로 사용하면 안 된다.
- `make test-grasp-pregrasp`: 9패키지 빌드 성공, 891개 테스트 통과.
  추가로 launch 설정 생성에서 21쌍 적용과 real-time 거부를 직접 확인.
  이번 요청에서는 설정/회귀 검증만 수행했으며 전체 수거 파이프라인은 재실행하지 않음.

### 기둥 충돌 무시 설정으로 전체 파이프라인 재시험

- 사용자 요청으로 headless Gemini Flash-Lite + SAM2 tracking, sim_speed_factor=1.0,
  기둥 충돌 무시 활성화 상태에서 실행. 9패키지 빌드 성공. 코드 변경 없음.
- 총 200.54초(약 3분 21초), 수거 검증 완료 1/4.
  레고: pick→transport→release→retreat→verify 모두 성공.
  `SIMULATION VERIFIED study_cafe_lego in lost_items_left`,
  최종 물체 중심 base_link=(-0.1076,0.1499,0.2468)m.
  시작부터 레고 수거 검증까지 149.63초.
- 이어 컵을 쓰레기로 분류해 오른팔 접근. `refreshed grasp approach trajectory
  execution failed: status=6 code=-4`로 중단. 컵 pick 시작 후 37.54초.
  right_arm_controller joint 1(right_shoulder_pitch_joint)의 목표 오차 -0.103321rad가
  goal tolerance 0.01rad를 넘고 2.004516초 유예도 초과하여
  GOAL_TOLERANCE_VIOLATED. 손을 닫는 파지 및 수거함 운반 전 단계의 실패다.
- 접촉 기록 전체에서 top_base_link 접촉 0건. 컵 접근 구간에서
  right_fixed_jaw_contact_2–cup_wall_30 접촉 normal force 최대 23.3148N,
  인접 cup_wall_29/31도 약 21N. 그리퍼 고정 턱과 컵 벽의 물리 접촉이 관측되며
  접근 pose/폭/offset 검증이 후속 과제. 기둥 오류 재발로 분류하지 않는다.
- 휴지/지갑은 컵 단계 중단 때문에 실행하지 못함.
  `artifacts/ignore_mast_retest_20260911/pipeline.log`, `contact_bag/`, stages 및
  서비스 요청/응답을 보존. GUI 미실행, 기록기를 포함한 실행 프로세스 종료 확인.

## README에서 분리한 과거 실행·설정 기록

다음 내용은 당시의 기록이며 현재 구현 설명이 아니다. 기존 근거를 보존하기 위해 옮겼다.

2026-09-07 headless 검증 attempt 33에서는 corrected moving-jaw 모델,
known-geometry updater와 명시 offset 0.030 m로 컵을 잡은 채 역접근에 성공했다.
실제 컵 중심이 약 168 mm 상승했지만 이후 추가 수직 LIN의 방향 불일치로
중단됐다. 상대 상승량 검증을 추가한 attempt 34는 지도 최신성 검사에서
막혔다. 4-worker self-mask를 적용한 35는 최신성/후퇴를 통과했고 실제 컵이
약 167 mm 상승했지만 held-view YOLOE 재검출 실패로 높이 검증에서 중단됐다.
640/960/1280과 cup/mug 표현의 저장 영상 시험도 이 재검출을 해결하지 못했다.
36에서는 현재 높이의 재관측 후보가 전체 물체를 화면에 담지 못했다.
낮은 관측 높이를 추가한 37은 실제 재관측 이동을 실행했으나 컵이 미끄러져
테이블로 떨어졌고 contact 검사에서 중단됐다. payload 속도를 낮춘 38/39는
각각 초기 IK 응답 초과/attachment 후 지도 갱신 공백으로 재관측에 미도달했다.
따라서 낮춘 속도의 지속 파지 효과도 아직 검증하지 못했다.
보유 검증을 생략하거나 confidence를 낮추지 않았다. 두 수거함으로 운반/놓기와
물리적 정착은 여전히 미검증이다. 관련 단위 테스트 통과를 수거 성공률로
해석하지 않는다. 실행별 기록은 `artifacts/sorting_20260907/README.md`에 있다.

현재 장면의 YOLOE label은 `cup`, `wallet`, `crumpled tissue`, `lego brick`이며
시뮬레이션 placement oracle의 body mapping도 이에 맞춰져 있다. 레고는 실제 2×4 크기
(몸체 31.8×15.8×9.6 mm, 돌기 포함 높이 11.4 mm)로, 기존 sorting의 50 mm 최소
파지 후보 폭 기준보다 작다. 크기를 부풀리거나 후보 기준을 자동으로 완화하지 않았다.
따라서 인식/분류 대상이어도 자동 집기는 제외될 수 있으며 별도의 소형 물체 파지 검증이
필요하다. 이 문서의 기존 성공·속도 측정은 교체 전 머그컵 장면 결과로, 새 종이컵과
휴지/레고의 수거 성공을 뜻하지 않는다. 이번 형상 교체에서는 모델 재학습이나 집기 성능
재측정을 하지 않았다.

CPU 시뮬레이터에서 열기 중 제어 주기 지연으로 추종 오차가 발생한 사례에
대응해 gripper motion은 8초로 설정하며, 경로 오차 기준은 그대로 유지한다.
action 응답 대기 시간도 설정한 motion duration에 비례한다.
첫 물체를 고르기 전에 초기 자세에서 그리퍼 열기 구간을 0.05 rad 이하
간격으로 충돌 검사하고 연다. sorting의 pregrasp 도착 후에는
열기 명령을 반복하지 않고 정지 확인·최신 인식·집기 재계산으로 진행한다.
따라서 기존의 중복 열기 8초가 제거되며, 놓기 시 개방은 유지된다.
이후 후보 선택은 열린 jaw의 실제 feedback을
사용한다. 이 이산 검사는 연속 충돌 검사의 대체물은 아니다.
재검출한 후보를 selector에 넘기기 전에 이전 snapshot의 target OBB를
제거한다. 같은 물체가 서로 다른 ID의 충돌체 두 개로 중복 등록되어
접촉 허용이 한쪽에만 적용되는 것을 방지한다.
닫기 직후 접촉 신호가 있더라도 settle 후 잔여 각도/속도 조건을 다시
확인하며, 유지되지 않으면 부착 OBB를 만들거나 lift로 진행하지 않는다.

2026-09-09 이관 회귀 검사: `make test-grasp-pregrasp`의 Python 798개와
C++ 44개가 통과했다(모델 정합성 29개 포함). Gemini + SAM2 tracking을 켠
headless 재실행은 레고 인식 → 오른팔 pregrasp → 손목 전환 → 접근/닫힘까지
진행했으나 `gripper retention not confirmed from joint feedback`로 중단됐다.
닫힘 목표 -0.30000rad 대비 실제 -0.29997rad로 접촉 정지 기준을 충족하지
못했다. 이 결과만으로 빗잡음·밀림·파지 판정 문제 중 원인을 확정할 수 없다.
들어올리기와 수거는 미검증이며 종료 과정의 MoveIt segmentation fault도 남아 있다.
로컬 재현 자료: `artifacts/migrated_parity_recheck_20260909/`.
