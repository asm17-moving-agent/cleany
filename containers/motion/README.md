# cuRobo physical selector runtime

`cleany-motion:jp6.2`의 physical selector는 GPU 후보 IK를 계산하고 호스트 MoveIt의 최종 상태 검사와 경로 계획을 호출한다. 모터 제어는 native controller가 소유한다. 기본 IK는 16 seed와 실패 후보의 32 seed 재시도를 사용하며 CPU fallback은 명시적으로 선택한다.

이미지는 `cleany-vision:jp6.2`와 cuRobo v0.7.8 commit `d64c4b005459db10c5dd867d8b30a87d5bda9bdb`를 사용한다. JetPack 6.2, CUDA 12.6, Python 3.10, CUDA PyTorch 2.8과 Orin SM 8.7을 전제로 한다. 호스트 native ROS 환경과 GPU 이미지의 Python 환경은 분리한다.

## 준비

```bash
make motion-container-image
make motion-container-prepare
make motion-container-check
make build-jetson
```

`prepare`는 interfaces, description, MoveIt config와 skill executor를 컨테이너에서 빌드한다. 저장소는 읽기 전용이며 출력과 CUDA cache는 checkout별 Docker volume에 둔다. 이미지 ID와 설치 입력 hash가 달라지면 `check`가 재준비를 요구한다. `CLEANY_MOTION_IMAGE`로 이미지 tag를 선택한다. 준비 실패 시 native로 자동 전환하지 않는다.

## 명시적인 physical launch

```bash
ros2 launch cleany_skill_executor physical_grasp_selection.launch.py \
  repository_root:="$PWD" motion_backend:=curobo_ik_moveit motion_runtime:=container
```

이 서버는 기존 `grasp_selection_server`와 같은 namespace에서 함께 실행하지 않는다. `motion_runtime:=auto|native|container`는 physical selector와 pregrasp 지원 경로에 적용한다. `auto`는 MoveIt이면 native, Jetson의 cuRobo이면 container를 선택한다. native cuRobo의 별도 Python은 `curobo_python`으로 지정한다. 기존 시뮬레이션 launch는 기존 backend를 유지한다.

launch prefix는 `physical_grasp_selection_server`만 받아 컨테이너의 `cleany_skill_executor.physical_grasp_selection_node`를 실행한다. ROS parameter YAML을 복사하고 native install과 저장소 경로를 mount 경로로 변환한다. domain, RMW와 DDS 설정은 전달하며 호스트 Python과 install 환경은 source하지 않는다.

selector와 `tools/motion-container exec`의 OpenBLAS 계산은 기본 1개 스레드를 사용한다. 인식과 경로 준비가 겹칠 때 CPU 경쟁을 줄이도록 Python 시작 전에 `OPENBLAS_NUM_THREADS`를 설정한다. `CLEANY_MOTION_BLAS_THREADS`로 양의 정수를 지정할 수 있으며 다음 컨테이너 실행부터 적용된다. ROS 콜백 스레드는 유지하며 PyTorch/OpenMP 전용 설정은 추가하지 않는다.

컨테이너는 호스트 network, IPC와 PID namespace를 사용하고 selector는 호스트 UID/GID로 실행한다. 시작한 컨테이너는 고유 이름과 owner label을 가지며 정상 종료, 오류와 signal 취소 시 해당 컨테이너만 정리한다. 종료 확인과 물리 정지 증거는 별개다.

```bash
tools/motion-container status --owner "$CLEANY_MOTION_OWNER"
tools/motion-container stop --owner "$CLEANY_MOTION_OWNER"
```

## 장치 없는 검증

`make test-jetson-tools`는 fake Docker로 플랫폼 선택, 외부 install 경로, parameter 복사, DDS 전달, signal 취소, 오류 전달과 정리 범위를 확인한다. 실제 GPU와 모델 준비는 별도다.

```bash
tools/motion-container exec -- python3 /workspace/cleany/containers/motion/smoke.py
```

위 smoke는 명시적으로 준비된 Jetson GPU 이미지를 사용하며 실물 모터에 명령하지 않는다. 검증 결과와 환경은 [EVIDENCE](../../docs/EVIDENCE.md), 로봇별 외부 프로파일은 [실행 안내](../../docs/RUNNING.md#jetson-실물-실행-경로)를 따른다.
