# 공통 파지 설정

dev-monitor의 `물체 1개 실행`은 `make manipulation-run`에서 자동 대상을 고른 뒤
`/manipulation/execute_skill`로 실행한다. 일반 물체 선택과 양팔 후보 검사를 유지한다.
운영 수거 파일은 `CLEANY_MANIPULATION_COLLECTION_CONFIG`로 지정하며 Jetson 기본 파일은
`configs/robot/jetson/collection.json`이다. CLI와 웹은 같은 선택기와 수거 실행기를 사용한다.

| 항목 | 계약 |
|---|---|
| 예측기 기준점 | 후보의 TCP는 `translation + depth_m * rotation[:, 0]`이다. 원본 후보를 수정하지 않는다. |
| 대칭 그리퍼 | 고정턱 TCP 이동, 추가 편향과 개구 중심 이동을 적용하지 않는다. 반대 파지 방향 중 현재 손목에서 회전량이 작은 방향을 먼저 검사한다. |
| 접근 중 개방 | 승인된 `open_joint_rad`까지 연다. 선택기의 상태·경로 검사도 같은 최대 개방 각도를 사용한다. 물체 폭은 수용 가능성 검사에 사용한다. |
| 준비 위치 Z | `pregrasp_z_offset_m`는 계획 좌표계 Z축의 미터 값이다. 준비 위치와 8/6/4cm 대체 준비 위치에만 더하며 최종 TCP와 접근 벡터는 바꾸지 않는다. Jetson 운영 파일은 `0.010`을 사용한다. |
| 복귀 기준 | 운영 파일의 `home_joint_rad: "startup"`은 시작 관절 피드백을 복귀 기준으로 저장한다. |
| GPU IK | `curobo_ik_moveit`, 배치 8개, CPU IK fallback 비활성 상태를 유지한다. |
| IK 끝점 여유 | `curobo_ik_endpoint_margin_rad: 0.1`은 활성 팔의 GPU 탐색 범위와 최종 해 선택에 적용한다. URDF, 모터 및 MoveIt의 물리 관절 한계는 바꾸지 않는다. |
| 경로 후보 수 | `maximum_path_candidates: 4`는 경로 검사에 진입할 서로 다른 후보·팔 조합을 제한한다. 먼저 검사한 조합의 지연 RRT 재시도는 유지한다. |

GPU 후보 생성은 활성 팔의 어깨 yaw 접근 범위와 끝점 여유를 함께 적용한다.
후보용 URDF의 해당 관절 한계만 좁히고 원본 모델과 반대 팔의 측정 상태는 유지한다.
초기 seed, 검사를 미룬 후보의 선택과 CPU 자세 보정에도 같은 범위를 적용하며,
범위를 벗어난 첫 후보 때문에 뒤의 유효한 후보를 버리지 않는다.

Jetson 운영 파일은 준비 위치에서 최종 파지 위치까지의 접근, 닫기, 들어올리기의 충돌 검사를 끈다.
최대 개방, 준비 위치로의 이동, 복귀와 놓은 뒤 이탈, 그리퍼 보유 물체의 OctoMap 검사는 유지한다.
선택 단계의 장면 충돌 검사와 경로 검사는 계속 수행한다. 모터 끝점 보정과 명목 개구 폭은 운영
파일의 값을 그대로 사용하며 측정하지 않은 CAD 개구 폭 곡선을 새로 적용하지 않는다.

`camera_preview.json`의 `rigid_accessory_files`는 왼쪽 손목의 고정 카메라 마운트 CAD 23개를
시각화와 충돌 모델에 등록한다. 왼쪽 V8 근사 형상 15개를 교체하며 오른쪽 형상은 유지한다.
등록 파일은 출처 SHA와 손목 부모 joint의 origin·axis를 검증한다. 이 형상은 자기 형상 필터와
일반 충돌 모델에서 함께 사용한다. 카메라의 광학 TF, TCP, 관절 보정, 필터 padding 또는 허용
충돌 목록을 변경하지 않는다.

모터 없는 GPU 검증은 `tools/validation/verify_dev_monitor_gpu_grasp.py`에 생성한 URDF와
실제 launch에서 합성한 파라미터 JSON을 전달한다. 양팔마다 알려진 FK 목표 8개를 CUDA로
계산하고 끝점 여유와 위치 오차를 검사한다. 이 검증은 controller를 시작하거나 장치를 열지 않는다.

GPU의 `joint_simplified_registered` 모드는 기존 링크의 형상 서명과 검증된 구를 유지한다.
왼쪽 `left_gripper_frame`만 현재의 모든 convex body를 감싸는 구를 만든 뒤, 포함 관계를
검사하며 설정된 개수로 합친다. 각 body의 모든 꼭짓점을 한 구가 포함하므로 내부 고체도
포함한다. 등록 형상 표시가 없거나 다른 링크의 서명이 달라지면 거부한다. 원본 CAD,
메시와 기존 검증 구 asset은 재생성하지 않는다.

## dev-monitor와 운영 인식 공유

Jetson dev-monitor의 저장된 실행 설정은 `CLEANY_COLLECTION_USE_EXISTING_PERCEPTION=true`를 사용한다.
이미 실행 중인 카메라와 `gemini-decision-1` 운영 인식을 공유하며, 파지 자세 계산은
`curobo_ik_moveit` 컨테이너에 맡긴다. 일반 물체 자동 선택은 유지한다.

| 경계 | 동작 |
|---|---|
| 시작 전 확인 | 현재 RGB·depth 영상과 InspectScene, ObserveObjectReference, ObserveWristTarget 서버를 읽기 전용으로 확인한다. 누락 시 모터 제어를 시작하기 전에 중단한다. |
| 헤드 카메라 | 지원 launch의 `start_head_camera=false`는 카메라 드라이버와 RGB-D TF bridge를 새로 시작하지 않는다. 기본값은 true다. |
| 시작·종료 소유권 | 공유된 인식·카메라 서비스를 시작하거나 종료하지 않는다. 관리 중인 런타임의 인식 소유권을 도중에 바꿀 수 없다. |
| 운영 인식 기능 | 사용자 service의 `50-dev-monitor-observation.conf`는 기존 실행 명령과 모델을 보존하고 물체 참조와 손목 관찰 기능만 켠다. |
| 독립 CLI | 환경변수를 생략하면 기존 컨테이너 인식 경로를 사용한다. 운영 인식을 공유할 때는 위 값을 명시한다. |

공유 제공자는 먼저 실행 중이어야 한다. `python3 tools/shared_perception_readiness.py`로
모터를 시작하지 않고 준비 상태를 확인할 수 있다. 기본 DDS domain은 호출 환경을 따른다.
`verify_motion_container_readiness.py`에 `--collection-config`를 추가하면 별도 DDS domain에서
공유 카메라 지원 launch 전체를 검사한다. 이 검사는 가상 관절 상태와 plan-only MoveIt만 사용한다.

공유 모드의 웹, 작업 프로세스와 수거 실행기는 저장된
`CLEANY_COLLECTION_SHARED_RMW_IMPLEMENTATION`을 ROS 초기화 전에 적용한다.
기본값 `rmw_fastrtps_cpp`는 현재 native 운영 제공자와 일치한다. 비공유 컨테이너 경로는
기존 CycloneDDS 설정을 유지한다. DDS 검색만으로 통신 성공을 판단하지 않는다.
준비 검사는 `/perception_inspector/get_parameters`의 실제 RPC 응답에서
`target_frame=base_link`, 물체 참조와 손목 관찰 활성 상태도 확인한다.

## 보정된 CAD 경로

`freecad_calibrated` 모델에서는 기존 GPU IK와 native MoveIt 충돌 판정을
연결한다. 캐시된 CAD primitive는 IK 비용에 사용하며, 최종 충돌 허용은
전체 CAD 형상으로 검사한다. 실제 손가락의 종속 좌표는 상태마다 다시
계산한다. 기존 폭 제어 곡선, 일반 물체 자동 선택과 단계별 검사 설정은
모델 연결만으로 변경하지 않는다. 상세 계약은
[CAD 파지 모델](../../cleany_moveit_collision/docs/calibrated-cad.md)을 따른다.
