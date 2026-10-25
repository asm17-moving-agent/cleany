# Isaac Sim 통합 검증 계획

## 기준과 범위

2026-10-24 원격 `main`의 `eb2c56a190ad9fbb52cb104ff55282e955a56fcb`를
`codex/isaac-mission-e2e` worktree로 분리했다. 기존 main의 미커밋 변경과
형제 isaac-training 저장소는 수정하지 않는다. KB는 읽기만 한다.
사용자가 요청한 Isaac 통합 검증을 수행하며, KB의 기존 Gazebo/MuJoCo 역할을
제품 결정으로 수정하지 않는다.

## 원격 main 초기 조사 결과

| 구성 | 현재 구현과 재사용 지점 | 통합 공백 |
|---|---|---|
| 로봇 | cleany_description의 XLeRobot URDF/xacro, MJCF, 메시, 양팔 5축과 그리퍼, head pan/tilt, 4륜 메카넘 | Isaac 변환, 관성/충돌/관절 및 센서 frame 확인 |
| Gazebo | cleany_gazebo_sim의 스터디카페/시설 모델, LiDAR, depth, base, 지도 및 목표 | Isaac 장면과 지도 좌표 일치 |
| MuJoCo | cleany_mujoco_sim의 ros2_control, RGB-D, 물체/수거함, 안착 평가; cleany_mujoco_observer의 물리/제어 주기 | 카메라와 독립 안착 검증의 Isaac adapter |
| Navigation | cleany_navigation의 AMCL/Nav2, 메카넘 제어/계획 plugin, 직렬 safety, collision monitor; 외부 backend launch 존재 | /clock, scan, depth, odometry, TF, cmd_vel backend |
| Mission | 실제 FSM/청소 BT, Nav2 adapter, arrival/stop/lifecycle gate, journal, offer/cancel/result | node.py가 perception/planner를 MockDesk에 고정. 조작 backend는 mock/action_mock만 허용 |
| 조작 Action | cleany_manipulation_bt의 BehaviorTree.CPP, 기존 sorting/MoveIt/gripper, 정지와 안착 근거 | 실행 profile=mujoco. Mission의 map_outcome은 mock만 허용하므로 endpoint remap만으로 연결 불가 |
| 인식/계획 | cleany_perception, scene_mapping, grasping, skill_executor의 RGB-D→scene→후보→MoveIt 경로 | Mission OperationPort 연결 부재. YOLOE 모델/text encoder/API 의존성 확인 필요 |
| MoveIt | cleany_moveit_config의 양팔 planning/controller map | Isaac FollowJointTrajectory와 joint state 연동, 이동 base TF 소유권 |
| 서버 연동 | control_bridge WebSocket↔Mission; logger/telemetry는 관측 | 1차 시험은 기존 OfferMission 서비스로 할당; Dashboard/외부 backend 검증과 구분 |
| 기존 Isaac 연구 | 형제 isaac-training의 URDF importer, 내장 ros2_control 및 물리 컵 테스트 | 고정 base와 별도 검증기이며 Mission E2E가 아님. main 1521328에 고정되어 있어 최신 main 근거로 사용할 수 없음 |

주요 근거는 [Mission 구성](../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/node.py),
[조작 결과 계약](../ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/manipulation.py),
[기존 통합 launch](../ros2_ws/src/cleany_bringup/launch/mission_sim.launch.py),
[Nav2 구성](../ros2_ws/src/cleany_navigation/cleany_navigation/launch_runtime.py),
[조작 BT launch](../ros2_ws/src/cleany_manipulation_bt/launch/manipulation_mujoco.launch.py)와
[조작 preflight](../ros2_ws/src/cleany_skill_executor/launch/study_cafe_nearest_grasp_demo.launch.py)다.

## 의존성에 따른 실행 순서

1. **환경과 기준 고정**: 현재 main과 외부 자산 hash, Isaac/ROS image ID, GPU/메모리,
   기존 프로세스 확인. 독립 build/output/cache, ROS domain과 잠금 사용.
2. **시뮬레이터 adapter**: 원본 URDF와 controller 설정을 가져와 한 장면에
   base/양팔/센서/책상/물체를 배치. 단일 clock, TF 소유권, cmd timeout,
   joint feedback와 FollowJointTrajectory를 검증. 이상적 base 구동을 사용할 경우
   메카넘 접촉 동역학 검증과 명확히 구분.
3. **실제 Mission와 Nav2 연결**: 기존 navigation/안전/도착 판정을 유지하고
   Mission 서비스로 할당→이동→책상 도착→대기 위치 복귀→완료를 실행.
   처음에는 기존 Action mock 조작/MockDesk 인식·계획을 명시적으로 사용.
   성공 결과에도 physical cleanup 미검증을 별도로 기록.
4. **인식/조작 경계 연결**: 기존 OperationPort와 Action transport를 재사용해
   실제 snapshot/계획/BT를 연결. profile 검증, 초기 팔 안전, 재시작 recovery,
   독립 안착 검증을 함께 처리. 센서 실패를 정답 위치나 mock 성공으로 대체하지 않음.
5. **실패 검증**: 주행 불가/timeout, 조작 실패, 취소, 피드백 또는 시계 중단.
   결과뿐 아니라 상태 전이, 최종 속도, 남은 Action, DB 일관성, 재할당 조건 확인.
6. **근거와 재현**: 명령, 코드/설정 hash, 이미지 ID, 원본 로그, journal,
   판정 JSON과 센서/물체 증거를 저장. 실행 범위를 RUNNING/TESTING/EVIDENCE에 연결.

## 단계별 완료 판정

- Adapter: 동일한 ROS frame/단위, 단일 publisher, 단조 증가 timestamp,
  command watchdog 정지, 팔 궤적 성공/취소와 fresh joint feedback.
- Mission 통합: 실제 mission_runtime와 Nav2 사용, 도착 gate 통과,
  RETURN_HOME 뒤 home 도착 및 SUCCESS. 조작 mock 여부를 report에 보존.
- 물리 정리: Isaac RGB-D로 식별한 물체를 기존 조작 소프트웨어로 이동하고
  독립 안착/정지 확인. 물체 강제 이동이나 attachment는 성공 근거로 금지.
- 실패: 실패 code와 상태, 정지 확인/불확실 구분, 성공 오보고 및 자동 재개 없음.

## 실행 환경 발견

호스트는 EndeavourOS이고 native ROS 설치가 없다. NVIDIA RTX 5070 12GB와
Isaac Sim 6.1 이미지, ROS Humble/MoveIt 이미지가 존재한다.
기존 이미지에는 전체 Nav2와 py_trees가 없어 별도 ROS 이미지를 구성한다.
기존 Isaac 학습/조작 실행은 종료하지 않는다. domain 73은 기존 조작이 사용한다.
이 작업은 별도 domain 174와 전용 캐시/컨테이너 이름을 사용한다.

NVIDIA의 [ROS 2 control 설명](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/ros2_tutorials/robot_control/tutorial_ros2_control.html)과
[ROS 2 노드 API](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.ros2.nodes/docs/index.html)를 참고하되,
설치된 6.1 API와 실제 실행으로 확인한다.

## 기본 mock 경로와 센서 통합의 경계

`--cleanup`을 지정하지 않은 기본 실행 경로는 1–3단계와 주요 실패 검증을 구현한
**mock 포함 Mission 통합**이다.
원본 로봇의 base, 양팔, head 형상과 PhysX LiDAR를 합성 방 또는 기존
Facility 18F 전체 층에 배치한다. Facility 경로는 원본 생성기의 벽, 문, 책상,
의자와 천장 충돌 형상을 보존한다. 합성 방의 물리 payload와 Facility 가구는
mock 수거에 맞춰 강제로 이동하지 않는다. 오른팔의 궤적 실행과 취소는 미션 시작 전 별도로 검증한다.
MoveIt 의존 패키지는 빌드하지만 실제 planning/물리 파지는 실행하지 않는다.

4단계의 `--cleanup` 경로에는 다음 연결을 구현했다. 연결 코드와 단위 검사
통과는 물리 수거 성공과 구분하며, 실행 결과는 EVIDENCE에서 확인한다.

1. Isaac RGB-D와 CameraInfo, 실제 frame/시간 동기화를 기존 perception 및
   scene mapping에 공급하고 모델/text encoder/API 자산을 확인한다.
2. Mission의 MockDesk 생성부를 기존 OperationPort 구현 선택으로 바꾸고,
   perception snapshot과 실제 조작 Action의 object/snapshot ID를 일치시킨다.
3. `map_outcome`의 mock 전용 검증을 실제 profile 계약으로 확장한다. 기존 BT의
   MuJoCo 전용 preflight, 초기 팔 안전, recovery와 placement verifier를
   simulator adapter 경계로 분리한 후 기존 MoveIt/ros2_control 경로를 연결한다.
4. 정답 pose나 강제 attachment 없이 Isaac sensor와 독립 물리 관측으로
   파지, 이동, 놓기, 팔 복귀를 확인한 다음 동일 Mission 시나리오를 다시 실행한다.

기본 mock 시험은 LiDAR 안전 경로만 실행하므로 depth 장애물 감지, 센서/clock 단절 주입,
물리 조작 중 취소, 재시작 recovery, 실물 메카넘 동역학은 검증 범위 밖이다.
기존 mock fixture의 snapshot 목록은 정상 수거 순서에 고정되어 있으므로
조작 실패 시험은 기존 설정으로 재시도를 0회로 지정한다. 실제 파지 실패는
팔 복구 미확인으로 재관찰 전에 ERROR로 종료한다. 실패 후 재시도용 snapshot
계약은 별도 통합 공백으로 남긴다.
실행별 판정과 실패 원본은 [EVIDENCE](EVIDENCE.md)에 기록한다.

## Facility 18F와 지정 주행 브랜치

사용자 요청에 따라 시설은 기존 Facility 18F 생성기/지도/좌석을 사용한다.
주행 코드는 `codex/nav2-tuning-recovered-20261024`의 `69b25020`에서 주행 튜닝
변경만 가져온다. 실제 Mission과 나머지 ROS 컴포넌트는 기존 main 기준이다.
MPPIController, Smac Lattice, 전용 primitive와 필요한 build/runtime 설정을
연결하며 main의 GTest 헤더 우선순위 수정은 유지한다. 수입 파일과 원본 hash는
`tools/validation/isaac/navigation_source.json`에 기록한다.
같은 브랜치의 복구된 `mppi-lattice-r10cm-benchmark.original.yaml`에서 FollowPath
optimizer 설정을 가져와 출력 궤적 검증, 경로 샘플 128개, 진행 감시를 활성화한다.
프로파일 원본은 `tools/validation/isaac/profiles/`에 보존한다. 벤치마크의 wrapper,
파일 경로와 속도 상한은 제외하며, 패키지의 속도 상한 0.15m/s와 0.3rad/s,
도착 판정 0.03m/0.1rad, Mission 도착 gate와 Stop/Slow 영역을 유지한다.
후속 속도 개선 요청에 따라 Facility 전용 `profiles/facility18.yaml`은 병진 상한을
0.25m/s, 회전 상한을 0.5rad/s로 올린다. 회전 샘플 표준편차는 0.2,
TwirlingCritic 가중치는 2로 설정해 최종 정렬에서 회전 억제를 줄인다.
기존 adapter 상한과 wheel projection, Stop/Slow 영역은 유지한다.
Omni planner는 같은 native Smac을 상속하는 `SmacLatticePlanner`로 연결한다.
Nav2 1.1.20의 같은 격자/방향 bin 역추적 실패를 전체 swept footprint 검사로
보완하고, 그 밖의 탐색은 원래 Smac에 맡긴다.
시설 시작 자세는 기존 park 프로파일의 관절 값을 원본 URDF 한계 안으로 제한한다.
팔을 제거하지 않으며, 주행 중 park feedback 이탈도 검사한다. 초기 자세에서
접는 전환 동작과 self-collision 검증은 이 범위에 포함하지 않는다.

## 센서 기반 책상 정리 통합

참고 브랜치는 `feat/jetson-manipulation-integration`의
`575e95bbba3574cdf33d08c986515e70edc857a9`다. 해당 브랜치도 Mission Manager의
MockDesk를 교체하지 않으므로 전체 브랜치를 합치는 것으로 Mission E2E가 완성되지는 않는다.
실물 BT의 실행 프로필 분리와 관측 ID, 분류, 목적지 검사 계약을 참고하고, 기존
sensor-driven sorting, MoveIt, BehaviorTree.CPP 실행기를 Isaac 어댑터로 재사용한다.
Jetson의 hold-only 및 temporary return-to-pick 모드와 `ASSUMED` 안착은 이 검증의
성공 조건에 포함하지 않는다. 튜닝된 주행 코드와 다른 작업 디렉터리는 보존한다.

검증은 다음 의존 순서로 진행한다.

1. 실제 InspectScene snapshot과 track ID를 Mission에 전달하고 `isaac` 실행 결과를
   엄격히 검사한다. 새 관측에서 물체가 보이지 않는다는 사실과 조작 성공 기록을 구별한다.
2. Facility 18의 실제 로봇 optical frame에서 RTX RGB-D를 발행한다. 기존 인식 모델,
   추적, 선택 물체 재구성 및 grasp 후보 생성을 실행한다.
3. 기존 MoveIt와 ros2_control을 통해 접촉 파지, 들어올리기, 수거함 놓기, 팔 복귀를
   검증한다. PhysX 물체 좌표는 독립 안착 평가기에만 전달하며 조작 목표로 사용하지 않는다.
4. 실제 Mission Manager로 주행, 관측, 수거, 재관측, 복귀를 실행하고 취소와 인식 또는
   조작 실패를 검증한다. 이 단계가 통과하기 전에는 물리 정리 E2E 성공으로 보고하지 않는다.

`run.py --cleanup`은 별도의 모델 디렉터리, ROS 빌드 및 출력 디렉터리를 요구한다.
기본 실행과 기존 CPU/GPU 병렬 벤치마크는 명시적인 mock 경로를 유지한다.
물리 fixture는 선택된 책상의 물체 한 개와 기존 로봇 장착 수거함이다.
손목 RGB는 기존 `sorting_wrist_cameras.yaml`의 nominal CAD 장착값과
`wrist_camera.yaml`의 해상도/FOV를 재사용한다. 선택된 팔의 실제 RTX 영상과
CameraInfo를 기존 WristService에 전달한다. 손목 depth나 물체 좌표를 생성하지
않으며, 기존 handoff를 통과해야 한다. 파지 후 확인은 설정에 따라 기존 손목
검사 또는 머리 RGB-D 검사로 수행한다. `slender-cup-95mm` 프로필은 머리 RGB-D의
실제 재검출과 관측 점군의 높이/위치 연관 및 지속적인 그리퍼 접촉을 요구한다.
공중 물체 재구성에는 InspectScene의 `free_space=true`를 사용해 책상 지지면까지
형상을 늘리지 않는다. 재관측 자세에서 시각 확인이 성공한 경우에만 저장한 팔 자세로
복귀한 뒤 기존 수거함 운반을 실행한다. 머리 RGB-D는
장애물 지도와 작업 전후 재관측을 위해 계속 발행한다. 이 장착값은 실물 보정값이 아니다.
종이컵과 구겨진 휴지는 기존 MuJoCo tabletop_shapes 생성기와 layout 설정을
재사용한다. 휴지는 강체 convex hull이며 종이 변형을 검증하지 않는다. 좌석의
기존 주행 목표와 작업 접근 목표를 함께 기록한다. 이 fixture의 성공이 모든 물체나
모든 좌석의 조작 성공을 뜻하지 않는다.

모델 자산은 Git 밖에 보관한다. 일반 YOLOE-26l와 MobileCLIP2는
[Ultralytics 공식 배포](https://docs.ultralytics.com/models/yoloe/)를 사용한다.
API 키가 없는 환경에서는
[Qwen3-VL-4B-Instruct](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct)를
로컬 분류 제공자로 시험한다. GPU에서는 NF4 이중 양자화와 BF16 연산을 사용한다.
기존 GeminiClassifier의 번호가 표시된 이미지, 분류 지침과 응답 ID 검증을 유지한다.
로컬 제공자는 JSON 스키마 자체 대신 데이터 객체를 반환하도록 형식을 명시하며,
그대로 사용한 전체 프롬프트와 원본 응답을 함께 저장한다. 초기 SmolVLM-500M 시험은 작은 이미지와
프롬프트에 따른 분류 불일치가 있어 물리 성공 근거로 채택하지 않았다. 이는 Jetson의 튜닝된 YOLOE checkpoint나 Gemini와 같은
모델이 아니며 성능 동등성을 주장하지 않는다. 손목 검출이 비어 있으면 동일한
공식 모델로 180도 회전 영상을 한 번 추가 추론하고 bbox와 mask를 원본 픽셀
좌표로 복원한다. 검출 신뢰도나 기존 손목 대상 연결 기준은 변경하지 않는다. Head 검출이 비어 있으면 RGB의 네 사분면을 각각 0/90/180/270도로 추가 추론한다. 타일 경계에 닿는 mask는 버리고 원본 픽셀로 복원한 뒤 같은 class의 mask IoU 0.5 이상 중복만 제거한다. 원본 신뢰도, RGB-D 재구성 및 물체 연관 기준은 유지한다. 이 공개 모델 보완에는 추가 추론 시간이 들며, 비어 있지 않은 원본 검출에서 누락된 물체까지 찾아주는 기능은 아니다. 2B 시험에서 나온 일회용/재사용
판정의 모순도 실패 근거로 보존한다. 분류 형식 오류나 불명확한 물체는
`human_review`로 처리한다. 모델이 물체를 놓칠 가능성은 독립 fixture 평가에서 확인한다.

물리 정리는 기존 `cleany_scene_mapping/KnownGeometryOctomapUpdater`를 명시적으로 선택한다. 공통 launch는 MuJoCo를 직접 시작할 때만 이를 자동 선택하므로, 외부 Isaac에서는 명시하지 않으면 표준 updater가 실행된다. 기존 관측 형상 내부 잔상 정리와 점군 최신성 검사를 재사용하며 외부 장애물 충돌 검사는 유지한다.

물리 cleanup에서는 원본 `motors.json`의 armature, frictionloss, passive damping을 articulation joint에 이식한다. static/dynamic friction effort는 동일한 frictionloss 값이며 viscous 항은 USD의 degree 단위로 변환한다. 구동기 PD damping과 수동 관절 damping은 별도다. `physics-settings.json`과 `motor-mapping.json`은 PhysX runtime의 radian 단위 값과 원본을 비교한다. 다른 simulator와 완전히 같은 마찰 해석이나 실물 보정을 의미하지 않는다.

독립 안착 oracle은 컵 wall collision vertex와 바닥 cylinder 또는 휴지 collision vertex를 실제 물리 pose로 변환해 base_link 경계를 계산한다. 회전한 전체 외접 box의 불필요한 확장으로 바닥/벽 관통을 오판하지 않는다. 평가기는 놓기 이후 최소 표본 수와 전체 안정 시간을 모두 만족하는 구간에서 내부 포함, 최신성 및 이동량을 검사한다. 이 정답 형상은 인식과 경로 계획의 입력으로 전달하지 않는다.
