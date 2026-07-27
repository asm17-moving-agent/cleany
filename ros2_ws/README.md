# ROS 2 Workspace

Cleany의 로봇 엣지 시스템 코드가 들어가는 ROS 2 workspace다.

패키지는 `src/` 아래에 두고, 공통 인터페이스는 `cleany_interfaces`에서 먼저 정의한다.

## 개발 환경

Docker 기반 ROS 2 Humble 개발 환경을 기본으로 사용한다. 아래 명령은 repository root에서 실행한다.

```bash
./scripts/docker-build-humble.sh
./scripts/docker-up-humble.sh
./scripts/rosdep install --from-paths ros2_ws/src --ignore-src -r -y
```

## 빌드와 테스트

전체 workspace 빌드와 테스트:

```bash
./scripts/ros2-build
./scripts/ros2-test
```

변경한 영역부터 빠르게 확인할 때는 타깃 pytest를 사용한다.

```bash
./scripts/pytest-ros src/cleany_mission_manager/tests/test_mission_flow.py
./scripts/pytest-ros src/cleany_mujoco_sim/test/test_scene_loader.py
```

## 실행 예시

MuJoCo 시뮬레이터를 headless 모드로 실행한다.

```bash
./scripts/ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true
```

패키지별 topic, launch parameter, 추가 검증 명령은 각 패키지 `README.md`를 따른다.
