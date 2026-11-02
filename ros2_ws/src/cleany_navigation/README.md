# Cleany Navigation

내비게이션 구성과 매핑 세션 관리를 제공한다. 매핑의 준비, 저장과 종료 및
실패 로그 경로는 [매핑 규약](docs/mapping.md), ROS endpoint는
[인터페이스](docs/interfaces.md)에서 확인한다.

저장소 루트의 `make test-hardware-mapping-core`로 매핑 수명주기와 실패 보존을 검사한다.
메시지 변경 후에는 `make build-hardware-mapping`과 `make test-hardware-mapping`을 실행한다.
합성 입력을 이용하는 실행 검증과 재생 절차는 [검증 안내](../../../docs/TESTING.md)를 따른다.
