# 실행 가이드

할 일을 선택하고 준비 → 실행 → 관측 → 종료 순서로 진행합니다. 각 가이드는 같은 저장소의 명령과 설정을 기준으로 합니다.

| 하려는 일 | 가이드 | 확인할 결과 |
|---|---|---|
| 가상 환경에서 센서·주행 확인 | [시뮬레이션](how-to/simulation.md) | scan, odom, TF |
| LiDAR로 지도 생성·저장 | [매핑](how-to/mapping.md) | map.yaml, PGM, pbstream |
| 저장 지도에서 목표 주행 | [휴대폰 주행](how-to/phone-navigation.md) | 경로 미리보기와 정지 응답 |
| 물체 관찰·후보 계획·조작 | [조작](how-to/manipulation.md) | snapshot, 계획, Action 결과 |
| Backend 요청과 실행 연결 | [Mission](how-to/mission.md) | 요청 수락, 허가, 결과 |
| 센서 입력과 장애물 관측 | [센서](how-to/sensors.md) | 영상·점군·frame·시각 |
| MCU와 베이스 연결 | [베이스](how-to/base.md), [펌웨어](../esp32/README.md) | 휠 피드백과 STOP ACK |
| 기록과 화면 확인 | [관측·기록](how-to/monitoring.md) | Logger 세션과 Monitor 화면 |

## 공통 준비

명령은 저장소 루트의 Ubuntu 22.04 / Humble 터미널에서 실행합니다. [환경과 출력 경로](DEVELOPMENT_SETUP.md), [연동 계약](INTEGRATION.md)을 확인한 뒤, 각 터미널에 같은 DDS domain과 같은 worktree overlay를 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
```

Navigation 전용 빌드는 `print-navigation-install-base`로 overlay를 조회합니다. 빌드 때 바꾼 Make 변수는 경로 조회 때도 동일하게 전달합니다.
