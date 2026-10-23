# cuRobo 실물 파지 선택(Selector) 런타임

`cleany-motion:jp6.2` 컨테이너는 cuRobo를 활용하여 GPU 기반의 역기구학(IK) 파지 후보를 고속으로 계산하고, 호스트 환경의 MoveIt을 호출하여 최종 상태 충돌 검사 및 경로 계획을 수행합니다. 모터 제어는 호스트의 네이티브 컨트롤러(`cleany_feetech_hardware`)가 담당합니다.

- **기본 IK 동작**: 초기 16개 시드(Seed)로 계산하며, 실패한 후보에 대해 32개 시드로 재시도합니다. (CPU Fallback은 설정 시에만 동작)
- **실행 환경**: JetPack 6.2, CUDA 12.6, Python 3.10, PyTorch 2.8, Orin GPU (SM 8.7 기준). 호스트 ROS 2 환경과 컨테이너 내부 환경은 철저히 분리됩니다.

## 1. 빌드 및 준비

```bash
# 1. 모션 컨테이너 이미지 빌드
make motion-container-image

# 2. 인터페이스 및 패키지 빌드 (저장소 읽기 전용, 볼륨에 빌드 산출물 저장)
make motion-container-prepare

# 3. 설치 상태 및 이미지 무결성 점검
make motion-container-check

# 4. 호스트 Jetson 실물 패키지 빌드
make build-jetson
```

> [!NOTE]
> `prepare` 단계는 `cleany_interfaces`, `cleany_description`, `cleany_moveit_config`, `cleany_skill_executor`를 컨테이너 내에서 안전하게 빌드합니다. 입력 해시가 변경되면 `check` 명령어가 재준비를 요청합니다.

## 2. 실물 실행 (Physical Launch)

```bash
ros2 launch cleany_skill_executor physical_grasp_selection.launch.py \
  repository_root:="$PWD" motion_backend:=curobo_ik_moveit motion_runtime:=container
```

### 주요 Launch 파라미터

| 파라미터 | 옵션값 | 설명 |
|---|---|---|
| `motion_backend` | `curobo_ik_moveit` (기본) | GPU cuRobo IK + MoveIt 결합 백엔드 |
| `motion_runtime` | `container` / `native` / `auto` | 실행 환경 선택 (`auto`는 MoveIt이면 native, Jetson cuRobo면 container 자동 선택) |
| `curobo_python` | Python 실행 경로 | `motion_runtime:=native` 사용 시 별도 cuRobo Python 경로 지정 |

- **스레드 및 CPU 자원 제어**:
  - 파지 선택기와 `tools/motion-container exec`의 OpenBLAS 연산은 기본 **1개 스레드**로 제한됩니다. (인식과 경로 계획이 동시에 동작할 때 CPU 경합 완화)
  - 스레드 수는 `CLEANY_MOTION_BLAS_THREADS` 환경 변수로 조정할 수 있습니다.
- **컨테이너 생명주기 관리**:
  - 호스트 네트워크, IPC, PID 네임스페이스를 공유하며 호스트 사용자의 UID/GID로 실행됩니다.
  - 다음 도구 명령어로 상태 확인 및 안전 종료가 가능합니다:
    ```bash
    tools/motion-container status --owner "$CLEANY_MOTION_OWNER"
    tools/motion-container stop --owner "$CLEANY_MOTION_OWNER"
    ```

## 3. AnyGrasp와 GPU 자원 조율 (GPU Work Lease)

AnyGrasp(비전)와 cuRobo(모션)가 한정된 GPU 메모리를 안전하게 공유하도록 **파일 락(`gpu-work.lock`) 기반의 Lease 메커니즘**이 적용되어 있습니다.

1. **공유 잠금 파일**: `configs/robot/gpu-work.lock` 및 `.demand` 파일을 호스트에서 바인드 마운트하여 두 컨테이너가 동일한 상태를 감시합니다.
2. **우선순위 원칙**:
   - 실제 사용자 작업 요청(Foreground Demand)이 백그라운드 예열(Warmup)보다 항상 최우선권을 갖습니다.
   - cuRobo 예열 도중 실제 파지 요청이 들어오면 다음 CUDA 단계 경계에서 즉시 GPU를 양보하고 대기합니다.
3. **업데이트 주의**: `cleany_grasping` 모듈이 변경된 경우 `make motion-container-image motion-container-prepare`를 실행하여 컨테이너 환경을 갱신해야 합니다.

## 4. 모터 없는 가상 환경 검증 (Smoke Test)

실제 로봇 팔을 연결하지 않고 컨테이너 내부 환경만 점검합니다:

```bash
# 호스트 도구 가상 테스트
make test-jetson-tools

# 모션 컨테이너 내부 스모크 테스트 (GPU 연산 확인, 모터 구동 없음)
tools/motion-container exec -- python3 /workspace/cleany/containers/motion/smoke.py
```
