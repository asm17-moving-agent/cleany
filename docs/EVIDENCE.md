# 검증 범위와 결과

**작업 요청부터 현장 좌석 이동, 책상 정리, 복귀, 결과 보고까지 이어지는 전체 실물 미션은 검증이 남아 있습니다.** 아래 표는 각 환경에서 관찰한 결과와 그 결과가 뒷받침하는 범위를 요약합니다.

| 기능 | 시험 환경과 관찰 결과 | 검증 범위와 근거 |
|---|---|---|
| 센서와 로봇 모델 | 실물 Jetson에서 RGB-D 입력을 수신했습니다. 관절 상태의 시각에 맞춰 헤드·손목 카메라와 파지 프레임의 TF를 조회했습니다. | 모델과 센서 연결을 확인했습니다. [RViz·TF 측정 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L7-L19) |
| 장애물 지도 | Jetson에서 기록한 점군과 TF를 재생했습니다. OctoMap 갱신 최적화 전후에 모든 재생 단계의 지도 해시가 일치했습니다. | 같은 입력에 대한 지도 보존을 확인했습니다. 현장 주행 중 장애물 회피는 별도 시험입니다. [측정 조건과 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L280-L298) |
| 거리 센서 | 단독 ESP32 보드에서 ToF 센서의 8×8 거리 프레임을 120초간 수신했습니다. | 보드의 출력과 전송 연속성을 확인했습니다. 로봇 장착 상태의 감지 범위와 정지 거리는 현장에서 측정해야 합니다. [펌웨어](../esp32/tof_vl53l7ch/README.md#실물-기록과-재생) |
| 물체 인식 | 운영 환경에서 헤드 RGB-D 한 프레임을 입력해 객체 6개가 포함된 응답을 받았습니다. | 입력부터 응답까지 연결한 시험입니다. 검출률과 3D 위치 정확도는 별도 평가가 필요합니다. [인식 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L324-L336) |
| 단일 물체 수거 | 고정 베이스 MuJoCo에서 컵 하나의 수거, 놓기, 팔 복귀와 정지를 확인했습니다. 파지 중 취소와 제어기 중단 시의 결과도 검사했습니다. | 단일 물체 시뮬레이션의 결과입니다. 여러 물체의 수거 성공률과 실물 팔 성능은 별도 평가 대상입니다. [시뮬레이션 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/ros2_ws/src/cleany_manipulation_bt/README.md?plain=1#L231-L306) |
| 미션 연동 | Gazebo/Nav2 주행과 Mock 조작 서버로 두 물체의 실행 ID, 재관찰과 미션 완료 보고를 확인했습니다. | 인식·계획·팔 동작에 모의 입력을 사용했습니다. [통합 시험 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L1022-L1044) · [재현 절차](implementation/how-to/mission.md#gazebo-주행과-mock-조작) |

현장 인수검사에서는 좌석별 접근 좌표와 장착 보정값을 확인한 뒤, 한 작업 요청의 이동·수거·복귀·결과 보고를 같은 미션 ID로 추적합니다. [보정 가이드](implementation/calibration.md) · [실행 가이드](implementation/running.md)

패키지별 검사 명령과 입력은 [검증 가이드](implementation/testing.md)에서 확인할 수 있습니다. 표에 인용한 실험의 커밋, 환경, 입력과 원본 위치는 [전체 실행 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md)에 보존되어 있습니다. 변경별 자동 검사 결과는 [GitHub Actions](https://github.com/asm17-moving-agent/cleany/actions/workflows/ci.yml)에서 확인할 수 있습니다.
