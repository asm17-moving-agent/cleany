# Jetson vision container

`cleany_perception`과 `cleany_grasping`을 JetPack 6.2 / CUDA 12.6 환경에서 함께 개발하고
실행한다. AnyGrasp는 컨테이너의 고정 MAC만 사용해 feature ID를 만들며 host의 Ethernet,
Wi-Fi, `docker0`, 다른 container의 `veth` 변화와 분리된다.

## 배포 identity

기본 identity는 다음과 같다.

| 항목 | 값 |
|---|---|
| Compose project | `cleany-vision` |
| Network | `172.30.0.0/24` |
| Container IP | `172.30.0.10` |
| Container MAC | `02:42:ac:1e:00:0a` |

`make vision-init`이 만드는 `.env`는 Git에서 제외된다. AnyGrasp license 신청 뒤에는
`VISION_MAC_ADDRESS`를 변경하지 않는다. 여러 robot을 같은 L2 network에 배포할 때는
license 신청 전에 robot마다 서로 다른 locally administered MAC과 subnet을 정한다.

`network_mode: host`를 사용하면 host의 가변 MAC들이 다시 AnyGrasp fingerprint에 들어가므로
사용하지 않는다.

## Host 준비

Jetson에는 JetPack 6.2와 NVIDIA Container Toolkit이 설치되어 있어야 한다.

```bash
uname -m                         # aarch64
nvcc --version                   # CUDA 12.6
docker info
```

JetPack 6.2.1의 기본 kernel에는 `CONFIG_IP_NF_RAW`가 없어서 Docker Engine 28 이상이 bridge
endpoint를 만들지 못한다. 이 Jetson에서는 최초 한 번 다음 host 설정이 필요하다.

```bash
make vision-host-setup
```

이 명령은 Docker가 공식 제공하는 `DOCKER_INSECURE_NO_IPTABLES_RAW=1` 호환 설정을 systemd
drop-in으로 설치하고 Docker daemon을 재시작한다. 이 설정은 Docker 전체에 적용되며,
`127.0.0.1`에만 publish한 port의 격리를 약화시킬 수 있다. Cleany vision service 자체는
port를 publish하지 않는다. 향후 Jetson kernel이 `CONFIG_IP_NF_RAW`를 제공하면 drop-in을
제거하고 Docker daemon을 재시작한다.

```bash
sudo rm /etc/systemd/system/docker.service.d/10-cleany-jetson-no-iptables-raw.conf
sudo systemctl daemon-reload
sudo systemctl restart docker
```

license와 model은 image에 넣지 않고 host에서 read-only로 mount한다.

```text
/home/cleany/.local/share/cleany/anygrasp/license/
  licenseCfg.json
  JeongHyeonLee.lic
  JeongHyeonLee.public_key
  JeongHyeonLee.signature
/home/cleany/models/anygrasp/checkpoint_detection.tar
/home/cleany/models/sam2/sam2.1_hiera_small.pt
```

초기 설정 파일을 만들고 실제 경로와 파일명을 확인한다.

```bash
make vision-init
${EDITOR:-nano} containers/vision/.env
make vision-config
```

`GEMINI_API_KEY`는 `.env`에 저장하지 않고 실행 shell에서 export한다.

## Image와 개발 container

Image는 CUDA 12.6, ROS 2 Humble, Jetson PyTorch 2.8, 수정 MinkowskiEngine,
AnyGrasp aarch64 dev SDK, GraspNet API와 SAM2를 설치한다. SDK와 주요 dependency는 commit
hash로 고정한다. Image build만 host network를 사용하며 runtime identity에는 영향을 주지
않는다. 첫 build에서는 CUDA extension compile에 시간이 오래 걸린다.

```bash
make vision-build
make vision-up
make vision-shell
```

workspace source는 `/workspace/cleany`에 bind mount되고 colcon `build-vision`,
`install-vision`, `log-vision`은 Docker volume에 둔다. host native build 결과와 섞이지
않는다.

```bash
make vision-shell
/opt/cleany/bin/build-workspace
python3 -m pytest -q \
  ros2_ws/src/cleany_perception/test \
  ros2_ws/src/cleany_grasping/test
```

## AnyGrasp feature ID와 license

feature ID는 반드시 최종 Compose container 안에서 만든다. 기존 host-native ID로 발급된
license는 이 container에서 사용할 수 없다.

```bash
make vision-feature-id
make vision-down
make vision-up
make vision-feature-id
sudo reboot
# 재접속 후
make vision-up
make vision-feature-id
```

세 출력이 같을 때 그 ID로 license를 신청한다. `anygrasp-feature-id` preflight는 container에
보이는 Ethernet MAC이 정확히 `VISION_MAC_ADDRESS` 하나인지 검사하고 다르면 실패한다.

새 license 네 파일을 host license directory에 교체한 뒤 검증한다.

```bash
make vision-license-check
```

## ROS 2 실행과 host 연결

한 container에서 perception과 grasping node를 실행한다.

```bash
export GEMINI_API_KEY='<api-key>'
make vision-run
```

container는 고정 bridge interface `eth0`만 사용하고 host gateway `172.30.0.1`을 CycloneDDS
peer로 지정한다. host에서 container node와 통신할 terminal은 다음 설정을 사용한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${PWD}/containers/vision/cyclonedds/host.xml"
export ROS_DOMAIN_ID=0
ros2 node list
```

공공 Wi-Fi가 multicast나 peer-to-peer traffic을 막더라도 host와 vision container 사이의
고정 bridge peer에는 영향을 주지 않는다. 다른 machine의 ROS graph까지 연결하는 topology는
DDS Router 또는 별도 robot network 설계가 필요하다.

## 제약

- AnyGrasp 공식 aarch64 SDK는 현재 시험 단계다.
- license MAC과 Compose network identity를 license 발급 후 바꾸지 않는다.
- Compose container에 network를 추가하면 feature ID가 달라질 수 있다.
- checkpoint와 license가 없더라도 node는 시작하지만 첫 grasp 요청은 model unavailable로
  실패한다.
- SDK update는 commit을 명시적으로 변경하고 feature ID와 license 검증을 다시 수행한다.
