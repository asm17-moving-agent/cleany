# 개발 도구

`tools/ros2/ros2-clean`은 해당 worktree의 ROS 빌드 출력을 정리합니다. `make clean` 실행 전에 삭제 대상 경로를 확인합니다. 홈, 저장소, 데이터 루트와 그 상위 경로는 삭제를 거부합니다.

| 작업 | 위치, 명령 |
|---|---|
| 시설 생성 | [Gazebo scripts](../ros2_ws/src/cleany_gazebo_sim/scripts/) |
| micro-ROS 준비, 검사 | [ESP32 scripts](../esp32/scripts/), `esp32/host_tests/` |
| Git wrapper 링크 갱신 | `make firmware-setup` |

실행과 검사 절차는 [테스트](../docs/TESTING.md), [ESP32](../esp32/README.md) 가이드를 따릅니다.
