# 모션 GPU 런타임

`cleany-motion:jp6.2`는 JetPack 6.2, CUDA 12.6과 Python 3.10에서 cuRobo IK를 실행한다.
MoveIt은 호스트에서 전체 장면의 충돌과 경로를 검사하고, 모터 제어는 호스트 controller가 담당한다.
일반 MoveIt backend는 이 컨테이너가 필요하지 않다.

## 준비

```bash
make motion-container-image
make motion-container-prepare
make motion-container-check
```

저장소는 읽기 전용으로 연결한다. 빌드와 캐시는 작업 트리별 Docker volume에 저장한다.
`prepare`는 인터페이스, 로봇 모델, MoveIt 설정, 파지와 실행 패키지 5개를 설치한다.
`check`는 이미지와 설치 입력의 hash를 대조한다. 입력이 바뀌면 다시 준비한다.
호스트의 MoveIt 준비는 [개발환경 안내](../../docs/DEVELOPMENT_SETUP.md#실물-모션-기반)를 따른다.

## 실행

```bash
ros2 launch cleany_skill_executor physical_grasp_selection.launch.py \
  repository_root:="$PWD" motion_backend:=curobo_ik_moveit motion_runtime:=container
```

| 파라미터 | 동작 |
|---|---|
| `motion_backend` | 기본 `moveit`, 선택 `curobo_ik_moveit` |
| `motion_runtime` | `auto`는 Jetson GPU backend에 컨테이너를 선택. `native`, `container`로 지정 가능 |
| `curobo_python` | 별도로 준비한 native GPU Python의 경로 |

호스트의 네트워크, IPC, PID namespace와 사용자 UID/GID를 사용한다.
`CLEANY_MOTION_BLAS_THREADS`는 OpenBLAS 스레드 수이며 기본값은 1이다.
`tools/motion-container status --owner OWNER`와 `stop --owner OWNER`로 해당 실행을 관리한다.

GPU 작업은 `configs/robot/gpu-work.lock`을 공유한다. 실제 요청은 백그라운드 예열보다
우선하며 진행 중인 CUDA 호출은 중단하지 않는다. 선택적으로 AnyGrasp를 실행할 때도
동일한 잠금을 사용한다. 기본 파지 후보 생성은 기하학적 방식이다.

## 검증

```bash
make test-motion-foundation MOTION_TEST_PATHS=tests/tools/test_motion_container.py
tools/motion-container exec -- python3 /workspace/cleany/containers/motion/smoke.py
```

GPU 회귀 검사는 `cleany_skill_executor/runtime_test/`에 있다.
실행 조건과 범위는 [테스트 안내](../../docs/TESTING.md#실물-계획과-실행-기반-검증)를 따른다.
