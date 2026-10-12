# 개발 도구

생성·환경 관리 도구를 역할별 폴더에 둔다.
도구의 단위 테스트와 런타임 검증 스크립트는 [`tests/`](../tests/README.md)에서 관리한다.

| 폴더 | 역할 |
|---|---|
| `micro_ros/` | ESP32 micro-ROS 의존성 준비와 고정 revision Git wrapper |
| `ros2/` | ROS 2 빌드 출력 정리 |

저장소 루트에서 micro-ROS 준비·검사는 `make firmware-setup`,
`make test-micro-ros-setup`을 사용한다. `make clean`은
`ros2/ros2-clean`을 실행하며 삭제 전 확인을 받는다.

도구의 단위 테스트만 실행하려면:

```bash
make test-tools
```

시설 asset·목표 생성 도구는 [`cleany_gazebo_sim/scripts/`](../ros2_ws/src/cleany_gazebo_sim/scripts/)에서 관리한다.

정리 도구는 빌드 경로가 홈·저장소·데이터 루트 또는 그 상위 경로인 경우 실행을 거부한다.
micro-ROS Git wrapper는 컴포넌트 루트와 그 하위에서 고정 커밋 처리를 적용한다.

기존 micro-ROS Git wrapper 링크는 `make firmware-setup`을 다시 실행하면
새 경로로 갱신된다.
