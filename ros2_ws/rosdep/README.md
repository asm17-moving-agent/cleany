# Cleany rosdep rules

`mujoco`와 `ultralytics`는 기본 rosdep 데이터베이스에 없으므로 native
환경용 fallback 규칙을 제공한다. 단, 이 규칙은 전이 의존성 hash를 고정하지
못하므로 저장소의 기본 Docker 경로에서는 사용하지 않는다.

저장소 wrapper는 두 key를 자동으로 건너뛰며, Docker 이미지에 이미 설치된
`requirements/ros2-humble-dev.txt` lock을 유지한다.

```bash
./scripts/rosdep install --from-paths ros2_ws/src --ignore-src -r -y
```

Docker를 사용하지 않는 native Ubuntu 환경에서 fallback이 꼭 필요한 경우에만
다음처럼 등록한다.

```bash
sudo sh -c 'echo "yaml file://'"$(pwd)"'/ros2_ws/rosdep/cleany.yaml" > /etc/ros/rosdep/sources.list.d/10-cleany.list'
rosdep update
```

native 설치는 완전 재현 경로가 아니며, 협업·CI·배포 기준은 Docker lock이다.
