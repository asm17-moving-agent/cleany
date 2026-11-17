# FreeCAD 링크와 회전축

`freecad_kinematic`은 저장된 CAD 자세에서의 각도 변화량으로 움직이는 형상
미리보기다. 양팔 10축, 헤드 2축, 바퀴 4축에
[PincOpen의 명목 핀 연동](freecad-gripper.md)을 추가했다. 독립 입력은 기존
16개와 캠 2개이며 종속 좌표를 포함한 전체 회전 좌표는 34개다.
[팁 맞물림](freecad-tip-mounts.md)은 네 고정 자식 링크로 표현하며 새로운
수동 회전 자유도를 추가하지 않는다.

## 추출 근거

축은 BREP의 해석적 원통면에서 구한다. 구동 혼과 서보의 중심선이 일치하고,
지지 혼이 있는 경우 그 중심선도 일치해야 선택한다. 중심선 일치의 수치 허용값은
0.02 mm와 0.0001 rad이며 물리적인 조립 허용오차나 제어 허용값이 아니다.
후보가 없거나 둘 이상이면 내보내기를 중단한다. 부품 이름의 Placement 축을
그대로 관절축으로 사용하지 않는다.

원본 부품의 소유 관계는
[`freecad_link_structure.json`](../config/geometry/freecad_link_structure.json)에,
원통면 번호, 반지름, 면적, 축과 비교 오차는
[`freecad_kinematics.json`](../config/geometry/freecad_kinematics.json)에 보존한다.
링크 연결은 원본 부품 그룹과 서보 케이스·구동 혼의 배치에 근거한 해석이며,
원본 파일에 조립 구속이나 실물 검증 결과가 추가된 것은 아니다.

| 움직이는 링크 | 부모 | 원본 근거 |
|---|---|---|
| 좌우 shoulder_yaw | base_link | Base_servo와 Yaw 구동·지지 혼 |
| 좌우 shoulder_pitch | 해당 shoulder_yaw | Yaw_servo와 Upper 구동·지지 혼 |
| 좌우 elbow_pitch | 해당 shoulder_pitch | Upper_servo와 Lower 구동·지지 혼 |
| 좌우 wrist_pitch | 해당 elbow_pitch | Lower_servo와 Wrist 구동·지지 혼 |
| 좌우 wrist_roll | 해당 wrist_pitch | Wrist_servo와 ToolDriveHorn |
| head_pan | base_link | HeadPanServoCase와 pan 구동·지지 혼 |
| head_tilt | head_pan | HeadTiltServoCase와 tilt 구동·지지 혼 |
| 4개 wheel | base_link | 원통형 wheel 외곽과 해당 motor 중심선 |

서보 케이스는 부모 링크, 그 서보의 구동 혼은 자식 링크에 둔다. 헤드는 원본의
단일 HeadRig 그룹에서 케이스, 브래킷, 혼과 카메라를 나누었다. 모든 입체 부품은
한 링크에만 속한다. 원본 `RightArm`은 로봇 오른팔이다.
헤드 팬 아래의 `HeadPanIdlerHorn`은 고정 브래킷 안에서 고정 서보 케이스에
닿고 움직이는 구동 혼과는 29.4mm 떨어져 있다. CAD 검사 모델에서는 이를
`base_link`에 배치하고 팬 중심선의 근거로 계속 사용한다. 실물 체결은 별도
확인 항목이다.

원본 바퀴 이름의 front/rear와 left/right는 화면 전방을 기준으로 한 ROS 방향과
다르다. 축과 위치에 따라 다음처럼 연결하며 모터 ID는 지정하지 않는다.

| 원본 이름 | 미리보기 위치 |
|---|---|
| Drive_rear_left | front_right |
| Drive_rear_right | front_left |
| Drive_front_left | rear_right |
| Drive_front_right | rear_left |

## 변화량 좌표

`cad_delta_*`의 단위는 rad이며 저장된 CAD 자세에서 추가로 회전한 각도다.
모든 변화량이 0이면 기존 `freecad_final`의 형상이 정확히 재현된다. 이는 물리적
관절 영점이나 encoder 0을 선택했다는 뜻이 아니다.

축 방향은 `base_link`에서 절댓값이 가장 큰 성분을 양수로 만드는 수치 규약이다.
서보의 양의 명령 방향은 미확정이다. 관절 프레임의 원점은 구동 혼 원통면의
표면 중심을 축에 투영한 점이다. 회전 중심선을 표현하는 좌표 선택이며 기준
각도나 실제 베어링 중심의 측정값을 추가하지 않는다.

URDF는 변화량 입력을 표현하기 위해 `continuous`를 사용한다. 이 형식은 실제
관절의 무제한 회전을 주장하지 않는다. 운용 한계가 미확정이므로 임의의
lower/upper, effort, velocity를 넣지 않았다. 이름도 기존 제어 관절과 다르며
이 모델에 기존 MoveIt 또는 hardware controller를 연결하지 않는다.

MuJoCo는 34개 mocap body를 FK 결과로 배치한다. 구동 관절, actuator, 질량과
접촉 응답을 추가하지 않는다. 따라서 모터로 구동하는 동역학 시뮬레이션이 아니라
관절축과 연결 구조의 운동학적 표시다.

## 미확정 값

| 항목 | 현재 상태 | 필요한 확인 |
|---|---|---|
| 관절 영점, encoder zero와 회전 부호 | 축마다 null | 영점 기준 자세와 실물 encoder 대응 |
| 가동 범위, 속도와 힘 제한 | 축마다 null | 기구 간섭, 배선, 모터 사양과 실물 시험 |
| PincOpen 구동 | 명목 폐쇄 링크와 팁 고정 자리 반영; 실물 보정·체결력은 미확정 | encoder와 개구 폭 대응, 유격, 체결·탄성 조건 |
| TCP와 파지 접촉 판정 | 미설정 | 기준점·접근축 정의와 손끝 접촉 시험 |
| 질량, 관성, 센서 외부 보정 | 기존 미확정 상태 유지 | 실물과 사양 근거 |

집게는 핀과 로드의 기하학적 거리로 연동한다. 실물 actuator와 접촉 응답은
아직 연결하지 않으며 `gripper_actuation=false`를 유지한다.

실행과 GIF 생성은 [실행 가이드](../../../../docs/RUNNING.md#freecad-형상-미리보기),
검사 범위는 [테스트 가이드](../../../../docs/TESTING.md#freecad-정적-기준-모델)를 따른다.
