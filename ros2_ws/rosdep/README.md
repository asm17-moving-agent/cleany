# Cleany rosdep 규칙

`mujoco`는 기본 rosdep database에 없으므로 이 workspace는 `cleany.yaml` 규칙을
제공한다. Ubuntu 22.04 VM에 ROS 2 Humble이 설치되어 있다는 전제에서 machine마다
한 번 등록한다.

레포지토리 루트에서 실행한다.

```bash
CLEANY_REPO_ROOT="$(git rev-parse --show-toplevel)"
echo "yaml file://${CLEANY_REPO_ROOT}/ros2_ws/rosdep/cleany.yaml" \
  | sudo tee /etc/ros/rosdep/sources.list.d/10-cleany.list
rosdep update
```

레포지토리 위치가 바뀌면 위 등록 명령을 다시 실행한다.

그다음 workspace 의존성을 설치한다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

레포지토리 루트의 `make deps`도 같은 rosdep install 명령을 실행한다.
