# Robot model 계약

CAD 파지와 미리보기는 `urdf/cleany.urdf`를 공유한다. 기존 14개 모터 좌표를 보정된 CAD
형상에 연결하며 CLI, MoveIt, cuRobo와 RViz가 같은 산출물을 읽는다.
기본 launch, 표시 상태와 갱신 절차는 [공통 CAD 런타임](runtime-model.md)을 따른다.

| 항목 | 계약 |
|---|---|
| 로봇 좌표 | `base_link`의 +X 전방, +Y 좌측, +Z 상방 |
| 광학 좌표 | +X 우측, +Y 하방, +Z 전방 |
| 모터 좌표 | 기존 양팔과 헤드의 14개 관절 이름, rad 단위와 보정 범위를 유지 |
| 집게 연동 | 16개 종속 관절을 기존 핀 연결 해석기로 계산하며 별도 모터로 제어하지 않음 |
| 집게 끝점 | 실제 최대 열림 0도와 닫힘 160도를 CAD의 양 끝 자세에 대응 |
| TCP | 현재 실물 프로파일의 손목 기준 119.775 mm 위치와 대칭 그리퍼 방향 |
| 충돌 | CAD 자기 충돌과 외부 장애물 검사를 유지하며 단계별 정책은 수거 프로파일에서 결정 |
| GPU | cuRobo IK와 native MoveIt 충돌 검사를 결합하며 CPU IK fallback을 기본으로 사용하지 않음 |

CAD 끝점 보간은 실측된 중간 개구 폭 곡선이 아니다. 원본 형상, 카메라 보정과
구동 좌표의 연결 및 검증 범위는 [CAD 파지 계약](../../cleany_moveit_collision/docs/calibrated-cad.md)을 따른다.
이 모델의 계산 일치도는 실물 TCP 정확도나 동역학 검증을 뜻하지 않는다.

`cleany.urdf.xacro`, control xacro와 `mjcf/cleany.xml`은 기존 시뮬레이션에서도 사용한다. 원본 CAD 입력과 과거 측정 기록은 재현 근거로 보존한다.

## 보존된 시뮬레이션 모델

기존 시뮬레이션은 `model_profile:=jetson_20261009`를 사용한다. Gazebo의 `cad_frame`은
이 xacro를 SDF로 변환하고, MuJoCo는 `mjcf/cleany.xml`을 읽는다. 실물 런타임의
보정된 CAD와 구분하며, 관절과 센서 좌표는 [이전 모델 reference](legacy-model-runtime.md)를 따른다.

실물 충돌 자산은 `meshes/cad_runtime_collision/`에서 관리한다.
시뮬레이션 모델 선택은 이 자산과 실물 `urdf/cleany.urdf`를 변경하지 않는다.
