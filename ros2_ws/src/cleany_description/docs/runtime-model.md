# 공통 CAD 런타임

설치된 `urdf/cleany.urdf`는 `tools/robot-model`, CAD 파지 프로파일의
MoveIt과 cuRobo, RViz가 공유하는 보정된 모델이다. `configs/robot/cad_grasp.json`으로 생성하며
기존 14개 모터 좌표와 최대 열림 0도에서 닫힘 160도 범위를 유지한다.
생성 파일의 해시는 `config/geometry/runtime_model.json`에 기록한다.

보정 입력 변경 뒤에는 Jetson에서 다음 순서로 공통 산출물을 갱신한다.
첫 명령은 모델 파일만 생성하며 모터를 조회하거나 움직이지 않는다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/validation/materialize_runtime_cad.py --repository "$PWD" --output /tmp/cleany-cad-materialized
make build COLCON_BUILD_ARGS='--packages-select cleany_description cleany_robot_interface cleany_moveit_config cleany_skill_executor --executor sequential'
./tools/robot-model check
```

`runtime_model_state`는 시리얼 장치와 제어기를 사용하지 않는다. 기본 입력은
전역 `/joint_states`의 모터 피드백이다. 출력은 실행 namespace 아래의
`joint_states`이며, 14개 모터와 계산된 16개 종속 관절을 포함한다.
원본 피드백과 출력이 같은 토픽이 되지 않도록 preview namespace를 지정한다.

| 상태 토픽 `status`의 `source` | 표시 자세 |
|---|---|
| `nominal_preview` | 아직 피드백이 없으며 모터 제한 안의 명목 자세를 표시한다. |
| `measured_feedback` | 14개 모터값이 모두 들어온 최근 피드백을 표시한다. |
| `stale_feedback` | 1초 넘게 새 피드백이 없으며 마지막 자세를 유지한다. |

RViz는 기본 `/runtime_model/robot_description`, namespace 안의 TF를 읽는다.
화면의 로봇이 움직여도 이 노드가 모터 명령을 생성하지 않는다.
`arm_preview.launch.py`도 같은 모델을 사용하며 직접 모터 버스를 열지 않는다.

`cleany.urdf.xacro`와 MJCF 모델은 기존 시뮬레이션 및 회귀 검사에서 사용한다.
실물 CAD 파지 프로파일은 장치별 보정 파일과 함께 명시적으로 선택한다.
그리퍼 끝점 대응과 충돌 판정은
[보정된 CAD 파지 모델](../../cleany_moveit_collision/docs/calibrated-cad.md)을 따른다.
