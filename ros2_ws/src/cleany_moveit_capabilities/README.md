# cleany_moveit_capabilities

MoveGroup 안에서 전체 장면에 대한 상태 검사를 일괄 수행한다.
`cleany_moveit_config`의 실물과 mock launch는 `cleany/StateValidityBatch`
capability를 기본으로 로드한다. 별도의 장면 복제 서버는 사용하지 않는다.

`/check_state_validity_batch` (`cleany_interfaces/srv/CheckStateValidityBatch`)는
최대 64개의 `RobotState`, 공통 group과 constraints를 받는다. 한 요청 동안
MoveIt planning scene과 OctoMap 읽기 잠금을 유지하며 모든 상태를 순서대로
검사한다. 각 diff는 같은 장면의 현재 상태에서 독립적으로 합쳐지고, live ACM,
attached object와 전체 mesh가 적용된다. 최초 실패까지의 validity와 해당 상태의
contacts를 반환한다. 계산 실패는 `success=false`이며 이동 허가가 아니다.
cost source는 반환하지 않는다. 충돌·제약의 유효성 기준은 native 상태 검사와 같다.

클라이언트 기본 청크는 32개다. 요청 사이에는 지도 갱신과 취소·실물 상태 감시가
계속된다. 샘플 간격이나 수를 줄이지 않으며, batch 결과를 다음 장면에서 재사용하지
않는다. 확장 서비스가 없는 다른 MoveIt 런타임에서는 기존 단일 상태 검사로 돌아간다.
실물 관리 runtime의 readiness는 이 서비스가 존재해야 통과한다.

```bash
make build-manipulation-speed
make test-manipulation-speed
```

C++ 검사는 합성 모델과 장애물을 사용해 샘플 누락, 최초 충돌 위치, live ACM,
attached payload와 잘못된 요청 처리를 검증한다. 실제 MoveGroup 서비스와 native
서비스의 결과 비교는 `cleany_moveit_config/test/test_state_validity_batch_runtime.py`로
검증한다. 이 검사들은 모터나 카메라 장치를 열지 않는다.
합성 실제 OctoMap과 attached payload 검사도 포함한다. payload/OctoMap 한 쌍을
ACM에서 허용해도 로봇 링크/OctoMap과 payload/명시 장애물 충돌은 계속 거부하며,
허용을 복원하면 payload/OctoMap 접촉도 다시 거부하는지 확인한다.
