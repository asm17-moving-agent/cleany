# MoveIt 배치 검사와 시간 배정

`cleany/StateValidityBatch`는 한 PlanningScene 읽기 잠금 안에서 전체 로봇 상태를
순서대로 검사한다. `/check_state_validity_batch`의 타입은
`cleany_interfaces/srv/CheckStateValidityBatch`이며 요청당 1~64개 상태를 받는다.
첫 실패까지만 검사하고 `valid`, `first_invalid_index`, 접촉 정보를 반환한다.
잘못된 입력과 계산 오류는 `success=false`다.

`cleany/RobustTimeParameterization`은 짧은 꺾임과 방향 반전을 처리하고 관절 한계,
속도, 가속도와 보간 경로 허용오차를 검사한다. 유효한 시간 궤적을 만들 수 없으면
계획 실패로 반환한다. 원본과 수정 범위는 [TOTG 출처](src/totg/README.md)에 있다.

충돌 형상과 시작 시 detector 확인은
[cleany_moveit_collision](../cleany_moveit_collision/README.md)이 담당한다.
이 패키지는 SolidFCL을 복제하거나 이동하지 않는다.

빌드와 검증은 `make build-motion-foundation`, `make test-motion-native`,
`make test-motion-runtime`을 사용한다. [검증 절차](../../../docs/TESTING.md#실물-계획과-실행-기반-검증)를 따른다.
