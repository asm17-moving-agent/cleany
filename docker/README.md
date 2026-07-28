# Docker 개발환경

Docker 환경은 Cleany의 기본 개발환경이 아니다. 팀의 공식 개발 기준은 Ubuntu 22.04
VM에서 ROS 2 Humble을 native로 사용하는 것이다.

환경 재현이나 격리된 테스트가 필요할 때만 레포지토리 루트에서 아래 도구를 사용한다.

```bash
./docker/scripts/build.sh
./docker/scripts/up.sh
./docker/scripts/rosdep.sh install --from-paths ros2_ws/src --ignore-src -r -y
./docker/scripts/bash.sh
```

컨테이너 안에서는 native 환경과 동일하게 `colcon`, `pytest`, `ros2` 명령을 직접
사용한다.

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

작업을 마치면 컨테이너를 제거한다.

```bash
./docker/scripts/down.sh
```

## 보조 명령

- `exec.sh`: 실행 중인 컨테이너에서 명령 실행
- `logs.sh`: 컨테이너 로그 확인
- `rebuild.sh`: 컨테이너를 제거하고 이미지와 컨테이너 다시 생성
- `rosdep.sh`: apt index를 갱신하고 컨테이너 안에서 root 권한으로 rosdep 실행
- `pyright.sh`: Docker 안의 Pyright language server 실행

이미지와 컨테이너 이름은 각각 `CLEANY_DOCKER_IMAGE`,
`CLEANY_DOCKER_CONTAINER` 환경 변수로 변경할 수 있다.
