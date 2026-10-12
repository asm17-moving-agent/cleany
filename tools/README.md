# 개발 도구

- ROS 출력 정리: tools/ros2/ros2-clean, `make clean` 실행 전 확인
- 보호 경로: 홈·저장소·데이터 루트와 상위 경로는 삭제 거부
- 단위 검사: `make test-tools`
- 시설 생성: [Gazebo scripts](../ros2_ws/src/cleany_gazebo_sim/scripts/)
- Micro-ROS 준비: [ESP32 scripts](../esp32/scripts/), 검사: esp32/host_tests/
- Wrapper 링크 갱신: `make firmware-setup`
- 안내: [테스트](../docs/TESTING.md), [ESP32](../esp32/README.md)
