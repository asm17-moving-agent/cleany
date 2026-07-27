# 개발 플랫폼 지원 범위

Cleany의 공통 기준은 ROS 2 Humble API와 repository lock이다. 운영체제별 GPU,
그래픽, DDS 제약이 다르므로 모든 호스트에서 동일한 실행 수준을 약속하지 않는다.

| 환경 | 지원 수준 | 기준 경로 | 필수 검증 |
|---|---|---|---|
| Ubuntu 22.04 x86_64 | 전체 개발 | Humble Docker 또는 native | build, pytest, MuJoCo, CPU/NVIDIA inference |
| Windows 11 | WSL2 개발 | Ubuntu 22.04 WSL2 + Docker | build, pytest, MuJoCo; NVIDIA 사용 시 CUDA smoke |
| macOS Apple Silicon | core/sim 개발 | native Python + MuJoCo | core pytest, scene load/render, CPU/MPS 실험 |
| macOS/Windows native ROS | 비지원 | Linux VM·WSL2·원격 Linux 사용 | 해당 없음 |
| Jetson Orin NX | 배포 | Ubuntu 22.04, JetPack 6.2.2, Humble | TensorRT export/runtime, 실제 센서, 장시간 부하 |
| CI linux/arm64 | packaging 검증 | ARM64 container build | dependency resolution, import, ROS package build |

## 공통 규칙

- 개발 Docker는 `requirements/ros2-humble-dev.txt`, Jetson은
  `requirements/jetson-jp622-arm64.txt`의 hash lock을 사용한다.
- OS별 native `pip install ultralytics`는 공식 재현 경로가 아니다.
- MuJoCo renderer는 호스트 GPU에 따라 선택한다.
  - NVIDIA Linux/WSL2: `MUJOCO_GL=egl`
  - macOS: 기본 Cocoa renderer 또는 headless test
  - GPU 경로가 불안정한 환경: CPU inference와 scene compile을 최소 게이트로 사용
- Windows native와 macOS native에서 ROS 2 Humble 전체 graph를 공식 지원하지
  않는다. ROS 통합 작업은 WSL2, Linux VM, 원격 Linux 또는 개발 컨테이너에서
  수행한다.
- TensorRT engine은 공유하지 않는다. 실제 배포할 Orin NX와 동일한 이미지에서
  생성하고 manifest를 함께 보관한다.

## 플랫폼별 완료 증거

플랫폼 지원을 추가하거나 올릴 때는 사용한 OS/architecture, Docker image
digest, dependency lock revision, 실행 명령과 결과를 PR 또는 검증 기록에 남긴다.
“코드상 가능”만으로 지원 상태를 올리지 않는다.
