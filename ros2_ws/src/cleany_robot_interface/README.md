# cleany_robot_interface

Feetech 모터 상태 조회, 안전 가드(Controller Guard), 손목 RGB 카메라 제어 및 실물 제어 실행을 담당하는 패키지입니다.
(Mission Manager의 범용 Mock/Sim/Real 어댑터 전체를 포괄하는 패키지는 아닙니다.)

## 1. 사전 준비 및 설정

실행 전 로봇별 엔코더 매핑, 캘리브레이션 파일 및 컨트롤러 한계 설정을 준비해야 합니다.
- **기본 모델 및 카메라 보정**: `cleany_description` 패키지의 `jetson_20261009` 프로파일을 기준으로 사용합니다.
- **단위 검증 주의**: 소프트웨어 및 가상 단위 테스트 통과 결과가 실제 하드웨어의 절대 보정 정확도나 파지 성공을 보장하는 것은 아닙니다.

## 2. 관련 가이드 링크

- **상세 인터페이스 및 토픽 규격**: [interfaces.md](docs/interfaces.md)
- **실물 로봇 구동 절차**: [RUNNING 가이드](../../../docs/RUNNING.md#jetson-실물-실행-경로)
- **하드웨어 없는 가상 검증**: [TESTING 가이드](../../../docs/TESTING.md#jetson-실물-변경-검증)
- **가상 시리얼 제어기 검사**: `runtime_test/` 아래의 PTY 테스트는 실제 하드웨어 포트를 열지 않고 가상 시리얼로 안전하게 동작을 검증합니다.
