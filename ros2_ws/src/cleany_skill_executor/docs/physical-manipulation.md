# 명시적으로 선택하는 실물 조작 경로

실물 파지 선택은 `physical_grasp_selection_server`와 `physical_grasp_selection.launch.py`를 사용한다. 구현은 `physical_grasp_selection_node.py`, `physical_moveit_adapter.py`, `core/physical_grasp_selection.py`에 있다. 기존 `grasp_selection_server`와 시뮬레이션의 MoveIt adapter는 기존 계약을 유지한다. 같은 namespace에서 두 선택 서버를 함께 기동하지 않는다.

실물 경로는 MoveIt 단독 또는 cuRobo IK와 MoveIt 검증을 연결한 `curobo_ik_moveit` backend를 명시적으로 선택한다. GPU가 생성한 후보는 최종 상태와 경로 검증을 대신하지 않는다. 선택 결과와 trajectory 실행, 물체 접촉, 배치 확인은 구분한다.

`pregrasp_runtime.launch.py`와 `pregrasp_hardware_support.launch.py`는 이 선택 서버를 사용한다. 실물 MoveIt은 `cleany_moveit_config/physical_move_group.launch.py`가 materialized URDF와 controller 설정을 받아 시작하며 trajectory 실행 기본값은 false다. 일반 `move_group.launch.py`의 기본값과 시뮬레이션 동작을 변경하지 않는다.

수거 실행은 외부의 로봇별 JSON 프로파일이 필요하다. `collection_hardware.json`이나 시험을 완료했다고 미리 설정한 공용 파일을 배포하지 않는다. `CLEANY_MANIPULATION_COLLECTION_CONFIG` 또는 launch의 `collection_config`로 경로를 지정한다. 설정 검증은 commissioning, 관절, 그리퍼 mapping, 목적지와 모드의 일관성을 확인하며 실제 보정과 실물 검증을 대신하지 않는다.

`manipulation_test_client --auto-select`는 분류, 신뢰도, 깊이, 크기와 금지 목록 검사를 통과한 물체를 `DetectedObject2D.distance_m` 오름차순으로 선택한다. 실물 인식의 기준 좌표계는 `base_link`이며 같은 거리에서는 `object_id`가 작은 물체를 먼저 선택한다. `median_depth_m`은 카메라 깊이의 유효성 검사에 사용하며 우선순위 기준으로 사용하지 않는다. 목적지가 설정되지 않은 물체는 건너뛰고 다음 가까운 적격 물체를 선택한다.

감독 시험의 `pick_return_release`는 명시적인 임시 모드다. 가정한 배치는 `ASSUMED`로 보존하며 센서 확인으로 취급하지 않는다. 관련 JSON은 `test/data/supervised_collection_fixture.json`에 합성 회귀 입력으로만 둔다. 실행 launch는 이 fixture를 기본값으로 읽지 않는다.

들어올리기 후 `lift_height_correction_enabled=true`이면 목표 높이와 실측 관절 FK 높이의 부족분을 `lift_height_correction_step_m` 단위로 올림해 한 번만 추가 상승한다. 기본 단위는 0.005m이며, 0.011m 부족하면 0.015m를 추가한다. 물리 경로는 실측 TCP에서 시작하고, 유지 중인 모터 명령의 지지량은 별도로 보존한다. 보정 후 남은 높이 오차는 기록만 하며 재보정이나 높이 합격 판정 없이 보유 확인과 복귀 흐름을 계속한다. 기존 `lift_height_tolerance_m`, `lift_height_correction_max_attempts`, `lift_height_correction_min_progress_m`, `lift_height_correction_max_total_m` 값은 사용하지 않는다. 모터 fault, 최신 피드백, 접촉, 취소, 명령 경로와 XY 및 방향의 도착 검사는 계속 적용한다.

SDK와 모델 weight, AnyGrasp 라이선스, 실행 DB, 영상, 점군과 결과 파일은 clone에 포함하지 않는다. 모델의 버전과 hash, 필요한 설치 방법은 저장소에 유지한다. cuRobo 전용 비교 도구와 모델 비교 캠페인은 일반 실행 경로에 포함하지 않는다.

Controller limits와 motion monitor는 `CLEANY_FEETECH_CONTROL_CONFIG`, `CLEANY_HARDWARE_MONITOR_CONFIG`로 각각 명시한다. BT launch의 대응 인자는 `hardware_monitor_config`다. 시험용 YAML도 test data에만 두며 실행 기본값으로 사용하지 않는다.


## 파지 목표의 추가 삽입

수거 공용 설정 `collection_motion.yaml`은 `grasp_approach_offset_m: 0.020`으로
최종 TCP 목표를 후보의 접근 방향으로 20mm 더 이동한다. `make manipulation-run`과
dev-monitor의 수거 실행은 같은 설정을 사용한다. 이 값은 수거 후보 전체에 적용하며
TCP의 기구학적 정의나 기존 닫힘 방향 보정을 변경하지 않는다. 원래 삽입 깊이로
돌리려면 이 값을 0.0으로 설정하고 planning runtime을 다시 준비한다.

접근 준비 위치는 추가 삽입 전 목표를 기준으로 유지한다. 따라서 8cm, 6cm, 4cm
준비 자세에서 보정한 최종 목표까지의 거리는 각각 10cm, 8cm, 6cm다. 보정 목표로
IK, 상태 유효성, 그리퍼 닫힘 공간과 경로를 다시 검사하며 기존 목표에 도착한 뒤
검사 없이 추가 이동하지 않는다. 이 20mm는 요청한 삽입량이며 SDK `grasp_depth_m`을
자동 적용한 값이나 실물 파지 성공을 검증한 보정량은 아니다.

## 접근 준비 자세의 거리 대체

`grasp_ik_first=true`에서는 파지 자세를 먼저 구하고 같은 해를 시작점으로 접근 준비
자세를 검사한다. `pregrasp_offset_m`의 기본 거리는 0.08m다. 이 준비 자세의 IK가
없거나 자세 허용 오차를 넘으면 `pregrasp_fallback_offsets_m`의 거리를 순서대로
검사한다. 수거 프로필은 `[0.06, 0.04]`를 사용하므로 8cm, 6cm, 4cm 순서가 된다.
앞선 거리에서 통과하면 이후 거리는 검사하지 않는다. 대체 거리는 양수이며 기본
거리보다 짧고 순서대로 감소해야 한다. 목록이 비어 있으면 기존 단일 설정인
`pregrasp_fallback_offset_m`을 사용하며, 이 값도 0이면 대체 검사를 끈다.

대체 준비 자세도 기존 위치와 방향 허용 오차, 전체 장면 충돌 검사, 두 구간의 경로
계획과 경로 자세 검사를 통과해야 한다. 접근 통로 검사가 켜져 있으면 실제 선택한
준비 자세를 시작점으로 사용한다. 이 설정은 파지 후보, 목표 파지 자세와 controller
실행 계약을 바꾸지 않는다. 피드백의 `offset`은 추가 삽입을 포함한 준비 위치와
최종 목표 사이의 거리이고, `fallback`은 대체 준비 자세 사용 여부다.

## GPU 준비 작업과 실제 요청 우선순위

`gpu_lock_path`가 설정되면 AnyGrasp worker와 같은 호스트 lock 및 `.demand` 파일을
사용한다. cuRobo 예열은 GPU가 사용 중이거나 실제 요청이 대기하면 시작하지 않는다.
진행 중인 예열은 단계 사이에서 양보하고 초기화한 solver buffer를 보존한다.
실제 선택 요청은 foreground lease를 유지하며 취소와 action deadline을 확인한다.
이미 시작한 CUDA 호출 자체를 중단하지는 않는다. 기본 container 프로필은
`/workspace/cleany/configs/robot/gpu-work.lock`을 사용한다. native 실행은 같은 호스트
inode를 가리키는 경로로 덮어쓴다. 빈 경로는 GPU 조정을 끈다.
