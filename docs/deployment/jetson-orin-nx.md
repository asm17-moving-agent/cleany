# Jetson Orin NX 배포

## 고정 플랫폼

| 항목 | 버전 |
|---|---|
| Device | NVIDIA Jetson Orin NX |
| OS | Ubuntu 22.04 |
| JetPack | 6.2.2 |
| Jetson Linux | 36.5 |
| CUDA | 12.6 |
| TensorRT | 10.3 |
| cuDNN | 9.3 |
| ROS | ROS 2 Humble |
| PyTorch | 2.10.0 aarch64 wheel |
| Ultralytics | 8.4.107 |

호스트 기준은 JetPack 6.2.2/Jetson Linux 36.5다. NVIDIA NGC에는 현재
R36.5 `l4t-jetpack` 이미지가 없으므로 컨테이너까지 R36.5라고 주장하지
않는다. 컨테이너는 NVIDIA가 서명한
`nvcr.io/nvidia/l4t-jetpack:r36.4.0` arm64 digest
`sha256:34ccf0f3b63c6da9eee45f2e79de9bf7fdf3beda9abfd72bbf285ae9d40bb673`을
사용하고, TensorRT 10.3을 유지한다.

PyTorch와 Ultralytics를 포함한 Python 전이 의존성은
`requirements/jetson-jp622-arm64.txt`의 hash lock으로 설치한다. 해당
PyTorch/torchvision wheel은 고정된 Ultralytics JetPack 6 recipe가 사용하는
aarch64 asset이며 URL과 SHA-256을 lock에 함께 기록한다.

## 1. 호스트 준비

JetPack 6.2.2를 설치한 뒤 Docker와 NVIDIA Container Runtime을 구성한다.
제품 장치에서는 편의 설치 스크립트 대신 Docker 공식 apt 저장소를 사용한다.

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl nvidia-container
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME}") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo usermod -aG docker "$USER"
```

로그아웃 후 다시 로그인하고 플랫폼을 확인한다.

```bash
uname -m
tr -d '\0' < /proc/device-tree/model
source /etc/os-release && echo "$VERSION_ID"
cat /etc/nv_tegra_release
docker info --format '{{json .Runtimes}}'
```

기대값은 각각 `aarch64`, `Orin NX`가 포함된 모델명, `22.04`,
`R36 ... REVISION: 5.x`, `nvidia` runtime이다.

## 2. 실행 이미지 빌드

빌드는 실제 Orin NX에서 수행한다.

```bash
./scripts/jetson/build-image.sh
```

기본 이미지 이름은 `cleany:orin-nx-jp6.2.2`다. 배포 레지스트리를 사용할
때는 `CLEANY_JETSON_IMAGE`로 이름을 지정하고, 검증이 끝난 이미지는 digest로
기록한다. 빌드 스크립트는 이미지 생성 후 컨테이너의 TensorRT, CUDA,
PyTorch, Ultralytics 버전과 GPU 접근을 검사한다.

lock을 갱신할 때는 개발 호스트에 `uv`를 설치한 뒤 다음 명령을 사용한다.

```bash
./scripts/compile-requirements.sh
```

생성된 두 lock의 diff와 wheel 출처·hash를 코드 리뷰 없이 갱신하지 않는다.

## 3. TensorRT FP16 변환

TensorRT engine은 GPU 아키텍처와 TensorRT 버전에 종속되므로 실제 배포할
Orin NX에서 실행 이미지로 생성한다. 먼저 승인된 모델을 받는다.

```bash
cd ros2_ws/src/cleany_perception/models
curl -fL \
  -o yolo11n-seg.pt \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-seg.pt
echo "55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152  yolo11n-seg.pt" \
  | sha256sum --check
cd ../../../..
```

```bash
./scripts/jetson/export-tensorrt.sh \
  "$PWD/ros2_ws/src/cleany_perception/models/yolo11n-seg.pt"
```

같은 디렉터리에 `yolo11n-seg.engine`과
`yolo11n-seg.engine.manifest`가 생성된다. export는 모델 provenance 및
SHA-256을 검사하고, 임시 디렉터리에서 engine을 만든 다음 실제 TensorRT
추론 smoke test가 성공한 경우에만 최종 경로로 이동한다. 기본 설정은 640px,
batch 1, static shape, FP16이다.

메모리가 부족하면 workspace를 줄일 수 있다.

```bash
CLEANY_TRT_WORKSPACE_GIB=1 \
  ./scripts/jetson/export-tensorrt.sh /absolute/path/to/yolo11n-seg.pt
```

INT8은 calibration dataset과 정확도 회귀 검증이 필요하므로 기본 배포
경로에 포함하지 않는다. FP16 latency·메모리·정확도를 측정한 후 별도
단계로 도입한다.

## 4. Perception 실행

카메라 드라이버가 호스트 또는 다른 ROS 컨테이너에서 `/image_raw`를
발행하고 있어야 한다.

```bash
./scripts/jetson/run-perception.sh \
  "$PWD/ros2_ws/src/cleany_perception/models/yolo11n-seg.engine"
```

입력 토픽과 ROS domain을 바꾸려면 다음처럼 실행한다.

```bash
ROS_DOMAIN_ID=10 CLEANY_IMAGE_TOPIC=/camera/color/image_raw \
  ./scripts/jetson/run-perception.sh \
  "$PWD/ros2_ws/src/cleany_perception/models/yolo11n-seg.engine"
```

runtime은 engine SHA-256, 이미지 ID, 장치명, L4T, TensorRT, CUDA,
PyTorch, Ultralytics 버전을 manifest와 비교한다. engine이나 manifest가
다르거나 다른 이미지로 실행하면 시작하지 않는다.

기본값 `CLEANY_ROS_LOCALHOST_ONLY=1`은 같은 host network namespace의 ROS
프로세스만 discovery 대상으로 삼는다. 외부 장치와 DDS 통신해야 할 때만
`CLEANY_ROS_LOCALHOST_ONLY=0`을 명시하고 전용 VLAN, 방화벽, 고정
`ROS_DOMAIN_ID`, SROS2 정책을 함께 적용한다.

확인:

```bash
ros2 topic hz /detections
ros2 topic echo /detections --once
```

Jetson 설정은 annotated 이미지 발행을 기본 비활성화한다. 운영 중 시각화가
필요할 때만 별도 설정으로 활성화한다.

## 배포 검증

- `torch.cuda.is_available()`가 `True`인지 확인
- `trtexec --version`이 TensorRT 10.3 계열인지 확인
- engine manifest의 모델·engine SHA-256과 실행 이미지 ID가 일치하는지 확인
- 실제 카메라 해상도에서 `/detections` 주기와 end-to-end latency 측정
- `tegrastats`로 GPU·메모리·온도·전력 상태 기록
- 장시간 실행 중 카메라 중단과 재연결 동작 검증

## 보안 및 라이선스

- export는 network/host IPC 없이 실행하고 모든 Linux capability를 제거한다.
- runtime은 ROS DDS 때문에 host network를 사용하지만 host IPC는 사용하지
  않으며 `no-new-privileges`, read-only rootfs, `nosuid,nodev` tmpfs를 적용한다.
- 모델 디렉터리는 runtime에서 read-only로 mount한다.
- Ultralytics 코드와 공식 모델은 AGPL-3.0 또는 Enterprise 라이선스 대상이다.
  제품 배포 전 적용 라이선스를 사람 검토로 확정한다.
- 모델 provenance와 dependency lock은 보안 리뷰 대상이며 자동 갱신하지 않는다.

## 실제 Orin 승인 기준

macOS/Windows/일반 Linux에서는 ARM64 dependency resolution과 스크립트
검증까지만 수행할 수 있다. 배포 승인은 실제 Orin NX에서 아래가 모두
성공했을 때만 부여한다.

1. `./scripts/jetson/build-image.sh`
2. 승인된 `.pt`에서 FP16 engine export와 smoke inference
3. 실제 카메라 입력으로 `/detections` 및 선택적 `/detections_image` 확인
4. 30분 이상 부하 테스트와 `tegrastats` 기록
5. 카메라 중단·재연결 및 잘못된 manifest 거부 확인
