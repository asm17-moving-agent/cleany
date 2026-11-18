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

## 시뮬레이션 조립 모델

<figure class="contract-figure" data-visual-id="cad-assembly">
<img src="assets/cad-assembly.png" alt="외장 패널, 디스플레이, 양팔과 구동부를 포함한 최신 조립 CAD" loading="lazy">
<figcaption>Cleany-final.FCStd의 FreeCAD 조립 렌더. CAD의 등록 자세이며 encoder zero 자세나 실물 검증 화면은 아닙니다. <a href="../../../../docs/assets/visuals/imported.json">출처</a></figcaption>
</figure>

| 경로 | 현재 기준 |
|---|---|
| 시뮬레이션 TF, MoveIt | `urdf/cad_final/geometry.xacro` |
| Gazebo 기본값 | 같은 URDF를 변환하는 `cad_frame`; navigation 설정은 팔 제외 |
| MuJoCo | `mjcf/cleany.xml`; 같은 부품별 메시와 관절 배치 |
| 이전 장착 | `model_profile:=jetson_20261009`; 이전 URDF, MJCF와 보정 자료 보존 |
| 예전 Gazebo 평가 형상 | `robot_model:=legacy`를 명시할 때만 사용 |

[형상 등록 정보](../config/geometry/cad_final.json)는 CAD 해시, 부품별 메시 해시, 좌표 변환과 조립 자세를 기록합니다. 팔 장착점, 헤드, 그리퍼와 카메라 형상은 최신 CAD 기준입니다. 팔의 관절 방향과 이름은 유지하며 CAD 조립 자세를 관절 영점으로 바꾸지 않습니다.

## 시뮬레이션 좌표와 동작 범위

| 항목 | 계약 |
|---|---|
| Robot 축 | +X 전방, +Y 좌측, +Z 상방 |
| CAD → robot | X=−CAD Y, Y=CAD X, Z=CAD Z; mm→m |
| CAD 원점 → base_link | 위 회전 후 `(-0.000025, 0.000027, -0.284497)` m 이동 |
| Chassis 기준 높이 | 평지 정지 상태의 MuJoCo base 높이 0.38 m 유지 |
| Wheelbase / track / radius | 0.35 / 0.6038 / 0.0635 m; 실측 odometry 보정과 별도 |
| 팔 | 기존 5관절 이름과 회전 방향, 명목 제한 유지; 장착과 연결 위치를 CAD 축에 등록 |
| TCP | 최신 CAD 양쪽 fingertip 중심; 이전 `(0, -0.100, 0)` 값을 사용하지 않음 |
| 링크형 그리퍼 | CAD의 닫힌 링크 관계가 제공되지 않아 현재 조립 형상으로 고정; 명령 활성화 거부 |
| 질량과 관성 | 이전 모델의 추정값 유지; 새 부품의 실측 동역학으로 해석하지 않음 |

D435 preview의 CAD 좌표는 조립 원점 기준입니다. 위 `base_link` 이동을 적용하지 않은 값을 URDF 좌표와 섞지 않습니다. Gazebo의 LiDAR 높이 실험 프로필과 기존 실물 LiDAR TF는 독립된 센서 설정입니다.

## 시뮬레이션 카메라

<figure class="contract-figure" data-visual-id="wrist-axes">
<img src="assets/wrist-axes.png" alt="최신 CAD의 왼쪽 gripper 기준 손목 optical 축" loading="lazy">
<figcaption>최신 CAD 렌즈 유리 앞면에서 0.1 mm 바깥의 명목 optical 원점입니다. 실측 hand-eye 보정이 아닙니다. <a href="../../../../docs/assets/visuals/generated.json">입력과 출처</a></figcaption>
</figure>

| 경로 | 사용 좌표 |
|---|---|
| Head pan / tilt | CAD horn 축; zero에서 정면, +tilt는 아래 방향 |
| Head depth | CAD D435 명목 imager offset |
| 손목 MuJoCo / Gazebo | `cad_final.json`의 좌우별 렌즈 배치 |
| Sorting wrist TF | `cleany_mujoco_sim/config/sorting_wrist_cameras.yaml`의 같은 배치 |
| `jetson_20261009` 보정 | 같은 이름의 이전 모델 프로필에서만 허용 |

이전 v8 카메라 형상과 optical 보정은 이전 장착에 보존합니다. 최신 CAD에 이전 보정을 선택하면 Xacro가 실패합니다. 렌즈의 기계적 축 등록은 intrinsics나 실제 hand-eye 보정을 대신하지 않습니다.
