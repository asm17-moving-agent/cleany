# Cleany physical robot interface

Feetech 상태 조회, 상태 기반 controller guard, 손목 RGB 카메라와 명시적으로 선택하는 실물 제어를 제공한다. Mission의 공통 Mock/Sim/Real adapter 전체를 구현한 패키지는 아니다.

실행 전 로봇별 encoder mapping, 보정 파일과 controller 설정을 준비한다. 기본 모델과 카메라 보정은 `cleany_description`의 `jetson_20261009` 프로파일을 사용한다. 절대 보정 정확도나 실물 동작 성공을 단위 테스트 결과로 판단하지 않는다.

노드와 통신은 [interfaces.md](docs/interfaces.md), 실행은 [RUNNING](../../../docs/RUNNING.md), 장치 없는 검사는 [TESTING](../../../docs/TESTING.md)을 따른다. `runtime_test/`의 PTY controller 검사는 실제 USB 장치를 열지 않는 명시 실행 검사다.
