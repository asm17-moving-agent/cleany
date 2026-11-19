# CAD 런타임 모델

현재 기본 모델과 표시 계약은 [공통 CAD 런타임](runtime-model.md)을 따른다.
CLI, MoveIt, GPU IK와 RViz는 같은 보정된 CAD를 사용한다.
`cleany.urdf.xacro`와 기존 MJCF는 시뮬레이션 및 회귀 검사에 사용한다.
원본 CAD, 보정 입력과 과거 검증 기록은 재현 근거로 보존한다.

그리퍼의 실제 0~160도 범위와 CAD 끝점 대응은
[CAD 파지 모델](../../cleany_moveit_collision/docs/calibrated-cad.md)에 정의한다.
