# MoveIt 충돌 검사

`cleany/SolidFCL`은 로봇 메시의 닫힌 볼록 조각을 내부가 채워진 FCL 형상으로
검사한다. 조각 내부에 완전히 들어간 장애물도 검출하며, 서로 떨어진 조각의
틈과 링크별 배치, 충돌 허용 행렬을 유지한다. 열린 면이나 오목한 조각은
초기화 오류로 거부한다. 임의 CAD를 볼록 외피로 바꾸지는 않는다.

자기 충돌은 매 요청에서 링크 경계 상자를 계산해 검사할 부품 쌍을 줄인다.
조건부 접촉 허용, world, attached object와 거리 쿼리는 FCL 규약을 따른다.
이전 자세의 충돌 결과는 재사용하지 않는다.

`cleany/RequireCollisionDetector`는 MoveGroup 시작 시 요청한 detector가 실제로
선택됐는지 확인한다. 로드 실패로 다른 detector가 선택되면 프로세스를
종료한다. 이 패키지는 ROS endpoint나 모터 제어 기능을 추가하지 않는다.

MoveIt launch는 이 패키지를 의존하며 `collision_detector:=cleany/SolidFCL`을
기본값으로 사용한다. 표면 검사 대조에는 `collision_detector:=FCL`을 명시한다.
형상 계약은 [모델 문서](../cleany_description/docs/model-runtime.md#선택적-충돌-메시-평면화),
빌드와 검사는 [검증 절차](../../../docs/TESTING.md#충돌-메시와-내부-장애물-검사)를 따른다.
