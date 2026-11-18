# CAD 런타임 모델

현재 기본 모델과 표시 계약은 [공통 CAD 런타임](runtime-model.md)을 따른다.
CLI, MoveIt, GPU IK와 RViz는 같은 보정된 CAD를 사용한다.
`cleany.urdf.xacro`와 기존 MJCF는 시뮬레이션 및 회귀 검사에 사용한다.
원본 CAD, 보정 입력과 과거 검증 기록은 재현 근거로 보존한다.

그리퍼의 실제 0~160도 범위와 CAD 끝점 대응은
[CAD 파지 모델](../../cleany_moveit_collision/docs/calibrated-cad.md)에 정의한다.

## 보존된 시뮬레이션 모델

`legacy_runtime`은 `model_profile:=jetson_20261009`의 xacro를 실행한다.
기존 장착의 관절, 관성, 센서와 회귀 검사 기준은 [이전 모델 reference](legacy-model-runtime.md)를 따른다.

```bash
ros2 launch cleany_description description.launch.py \
  model_variant:=legacy_runtime model_profile:=jetson_20261009
```

| 진입점 | 역할 |
|---|---|
| `urdf/cleany.urdf` | 실물 CAD 파지, MoveIt와 RViz의 공통 보정 모델 |
| `urdf/cleany.urdf.xacro` | 기존 시뮬레이션의 head, wheel과 arm 모델 |
| `urdf/cleany_control.urdf.xacro` | 기존 시뮬레이션 팔 제어 모델 |
| `mjcf/cleany.xml` | 기존 MuJoCo scene 원본 |
| MoveIt `move_group.launch.py` | 기존 시뮬레이션 모델과 `joint_limits.yaml` 사용 |

`include_head_camera:=false`여도 고정 마스트는 남는다. `include_wheel_joints:=false`는
바퀴 형상과 고정 연결을 유지한다. 기존 장착의 보정은 `calibration_profile:=jetson_20261009`로
선택하며 실물 공통 CAD의 보정과 혼합하지 않는다.

실물 CAD의 원본, collision 자산과 갱신 절차는 [공통 CAD 런타임](runtime-model.md)을 따른다.
모델 회귀 검사는 `make test-description-models`로 실행한다.
