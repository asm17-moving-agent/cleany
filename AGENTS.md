# AGENTS.md

이 문서는 Cleany 구현 레포에서 AI 코딩 에이전트가 따라야 할 작업 규칙이다. 제품/기획 근거는 `docs/cleany-docs/` submodule의 KB를 우선 참고한다.

## 1. 저장소 성격

- 이 저장소는 끌리니(Cleany) 로봇 엣지 시스템 구현 레포다.
- 핵심 구현은 `ros2_ws/`의 ROS 2 workspace에 둔다.
- 기본 개발환경은 Ubuntu 22.04 VM의 native ROS 2 Humble이다. 신규 환경 구성은 `docs/DEVELOPMENT_SETUP.md`를 따른다.
- `docs/cleany-docs/`는 기획/예비설계 KB submodule이다. 비어 있거나 누락되었으면 먼저 실행한다.

```bash
git submodule update --init --recursive docs/cleany-docs
```

- `docs/cleany-docs/`를 수정해야 하는 명시 요청이 있을 때만 submodule 내부 파일을 편집한다.
- `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`, `__pycache__/`, `.pytest_cache/`, `.ruff_cache/` 같은 생성물은 편집하지 않는다.

## 2. KB 참조와 구현 문서 경계

제품 범위, 아키텍처, 안전, 하드웨어 또는 런타임 전제에 관한 KB 근거가 필요한 작업은 `docs/cleany-docs/AGENTS.md`와 그 안내 문서를 우선 따른다. KB를 수정할 때도 해당 submodule의 `AGENTS.md`를 우선 따른다.

### 구현 문서 갱신

- 구현 사실, ROS 인터페이스, 설정, 실행 및 검증 방법은 해당 패키지의 `README.md`가 관리한다. 이 내용이 코드 변경으로 달라지면 README를 같은 변경에 포함한다.
- 개발환경 설치와 system dependency 기준은 `docs/DEVELOPMENT_SETUP.md`에서 관리한다.
- KB에서 아직 검토 중인 전제는 코드나 구현 README에서 확정 사실처럼 표현하거나 하드코딩하지 않는다.

## 3. 코드 작성 규칙

- Python 코드는 type hint, 작은 dataclass/model, 명확한 port/interface를 선호한다.
- ROS 2 node와 순수 core logic을 가능하면 분리해 core logic은 pytest로 빠르게 검증 가능하게 한다.
- 패키지 경계를 넘는 의존은 명시적 interface/message/config를 통해 연결한다.
- 설정값은 코드에 숨겨 하드코딩하지 말고 `configs/mission/`, `configs/robot/` 또는 ROS parameter로 이동할 수 있게 작성한다.
- ROS 패키지 dependency를 추가하면 `package.xml`, `setup.py`/`setup.cfg`, 필요 시 `ros2_ws/rosdep/cleany.yaml`을 함께 갱신한다.
- 생성 파일, 대용량 asset, mesh 파일은 명시 요청 없이 재생성하거나 포맷하지 않는다.

## 4. 검증 규칙

- 코드를 바꿨으면 가장 작은 관련 테스트를 먼저 실행한다.
- FSM/core logic 변경: 해당 패키지 pytest를 우선 실행한다.
- ROS package manifest, launch, install data 변경: 관련 패키지 또는 workspace build를 실행한다.
- 여러 패키지 경계나 ROS interface를 바꿨으면 전체 workspace test까지 고려한다.
- 반복 빌드·테스트는 루트 Makefile을 사용하고, 세부 옵션이 필요하면 `ros2_ws/README.md`의 native 명령을 따른다.
- 공통 빌드·테스트·실행 명령은 `ros2_ws/README.md`, 패키지별 명령은 해당 패키지 `README.md`를 따른다.
- 검증을 실행하지 못했으면 최종 응답에 이유와 대신 확인한 내용을 적는다.

## 5. KB 관련 판단

구현 중 KB와 충돌하거나 추가 결정이 필요한 사항을 발견하면 임의로 확정하지 않는다. 관련 KB 문서를 참고하고, 필요한 경우 코드에는 최소한의 TODO 또는 주석으로 불확실성을 남긴다.
