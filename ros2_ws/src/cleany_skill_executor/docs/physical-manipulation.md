# 명시적으로 선택하는 실물 조작 경로

실물 파지 선택은 `physical_grasp_selection_server`와 `physical_grasp_selection.launch.py`를 사용한다. 구현은 `physical_grasp_selection_node.py`, `physical_moveit_adapter.py`, `core/physical_grasp_selection.py`에 있다. 기존 `grasp_selection_server`와 시뮬레이션의 MoveIt adapter는 기존 계약을 유지한다. 같은 namespace에서 두 선택 서버를 함께 기동하지 않는다.

실물 경로는 MoveIt 단독 또는 cuRobo IK와 MoveIt 검증을 연결한 `curobo_ik_moveit` backend를 명시적으로 선택한다. GPU가 생성한 후보는 최종 상태와 경로 검증을 대신하지 않는다. 선택 결과와 trajectory 실행, 물체 접촉, 배치 확인은 구분한다.

`pregrasp_runtime.launch.py`와 `pregrasp_hardware_support.launch.py`는 이 선택 서버를 사용한다. 실물 MoveIt은 `cleany_moveit_config/physical_move_group.launch.py`가 materialized URDF와 controller 설정을 받아 시작하며 trajectory 실행 기본값은 false다. 일반 `move_group.launch.py`의 기본값과 시뮬레이션 동작을 변경하지 않는다.

수거 실행은 외부의 로봇별 JSON 프로파일이 필요하다. `collection_hardware.json`이나 시험을 완료했다고 미리 설정한 공용 파일을 배포하지 않는다. `CLEANY_MANIPULATION_COLLECTION_CONFIG` 또는 launch의 `collection_config`로 경로를 지정한다. 설정 검증은 commissioning, 관절, 그리퍼 mapping, 목적지와 모드의 일관성을 확인하며 실제 보정과 실물 검증을 대신하지 않는다.

`manipulation_test_client --auto-select`는 분류, 신뢰도, 깊이, 크기와 금지 목록 검사를 통과한 물체를 `DetectedObject2D.distance_m` 오름차순으로 선택한다. 실물 인식의 기준 좌표계는 `base_link`이며 같은 거리에서는 `object_id`가 작은 물체를 먼저 선택한다. `median_depth_m`은 카메라 깊이의 유효성 검사에 사용하며 우선순위 기준으로 사용하지 않는다. 목적지가 설정되지 않은 물체는 건너뛰고 다음 가까운 적격 물체를 선택한다.

감독 시험의 `pick_return_release`는 명시적인 임시 모드다. 가정한 배치는 `ASSUMED`로 보존하며 센서 확인으로 취급하지 않는다. 관련 JSON은 `test/data/supervised_collection_fixture.json`에 합성 회귀 입력으로만 둔다. 실행 launch는 이 fixture를 기본값으로 읽지 않는다.

SDK와 모델 weight, AnyGrasp 라이선스, 실행 DB, 영상, 점군과 결과 파일은 clone에 포함하지 않는다. 모델의 버전과 hash, 필요한 설치 방법은 저장소에 유지한다. cuRobo 전용 비교 도구와 모델 비교 캠페인은 일반 실행 경로에 포함하지 않는다.

Controller limits와 motion monitor는 `CLEANY_FEETECH_CONTROL_CONFIG`, `CLEANY_HARDWARE_MONITOR_CONFIG`로 각각 명시한다. BT launch의 대응 인자는 `hardware_monitor_config`다. 시험용 YAML도 test data에만 두며 실행 기본값으로 사용하지 않는다.
