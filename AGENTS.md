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

- 실행, 테스트, 캘리브레이션 절차는 `docs/RUNNING.md`, `docs/TESTING.md`, `docs/CALIBRATION.md`가 관리한다. 인터페이스, 단위, 멱등성, 안전, 데이터 규약은 `docs/INTEGRATION.md`에서 연결한 패키지의 `docs/` reference가 관리한다. 코드 변경으로 달라진 문서를 같은 변경에 포함한다.
- ROS endpoint를 변경하면 해당 패키지 `docs/interfaces.md`를 갱신한다. 노드 추가, 삭제와 실행 파일 변경은 `docs/ROS_INTERFACES.md` 색인에도 반영한다.
- 구현 문서는 내용에 맞는 서술 형식을 선택한다. 역할과 정상 동작은 짧은 완전한 문장으로 설명하고, 흐름은 도식, 인터페이스, 단위와 조건은 표, 실행 절차는 번호와 명령 블록으로 정리한다. 목록은 독립적인 확인 항목에 사용한다. 나열에는 쉼표나 자연스러운 접속 표현을 사용하고 가운뎃점으로 단어를 연결하지 않는다. 고정 템플릿의 빈 항목, 코드 해설, 작업 일지, 중복 설명은 추가하지 않는다.
- 폴더마다 README를 만들지 않는다. 독립 배포, 사용 절차가 필요한 도구, 펌웨어와 asset 출처와 라이선스처럼 고유한 역할이 있는 경우만 둔다. 파일 목록, 코드 요약, 현재 테스트 점수, 작업 일지를 README에 복제하지 않는다.
- 과거 실행 근거는 `docs/EVIDENCE.md`에서 찾고 새 결과에는 commit, 환경, 입력, 실패, 원본 위치를 남긴다.
- 개발환경 설치와 system dependency 기준은 `docs/DEVELOPMENT_SETUP.md`에서 관리한다.
- KB에서 아직 검토 중인 전제는 코드나 구현 문서에서 확정 사실처럼 표현하거나 하드코딩하지 않는다.

## 3. 코드 작성 규칙

- Python 코드는 type hint, 작은 dataclass/model, 명확한 port/interface를 선호한다.
- ROS 2 node와 순수 core logic을 가능하면 분리해 core logic은 pytest로 빠르게 검증 가능하게 한다.
- 패키지 경계를 넘는 의존은 명시적 interface/message/config를 통해 연결한다.
- 설정값은 코드에 숨겨 하드코딩하지 말고 패키지 `config/`, `configs/robot/` 또는 ROS parameter로 이동할 수 있게 작성한다.
- ROS 패키지 dependency를 추가하면 `package.xml`과 `setup.py`/`setup.cfg`를 함께 갱신한다. 외부 의존성은 공식 rosdep 키를 사용하며, 공식 규칙으로 설치할 수 없는 경우에만 별도 설치 방법을 검토한다.
- 생성 파일, 대용량 asset, mesh 파일은 명시 요청 없이 재생성하거나 포맷하지 않는다.

## 4. 검증 규칙

- 코드를 바꿨으면 가장 작은 관련 테스트를 먼저 실행한다.
- FSM/core logic 변경: 해당 패키지 pytest를 우선 실행한다.
- ROS package manifest, launch, install data 변경: 관련 패키지 또는 workspace build를 실행한다.
- 여러 패키지 경계나 ROS interface를 바꿨으면 전체 workspace test까지 고려한다.
- 반복 빌드와 테스트는 루트 Makefile을 사용하고, 세부 옵션은 해당 launch의 `--show-args`와 Makefile을 따른다.
- 공통 빌드는 `docs/DEVELOPMENT_SETUP.md`, 실행은 `docs/RUNNING.md`, 검증은 `docs/TESTING.md`를 따른다.
- 검증을 실행하지 못했으면 최종 응답에 이유와 대신 확인한 내용을 적는다.

## 5. KB 관련 판단

구현 중 KB와 충돌하거나 추가 결정이 필요한 사항을 발견하면 임의로 확정하지 않는다. 관련 KB 문서를 참고하고, 필요한 경우 코드에는 최소한의 TODO 또는 주석으로 불확실성을 남긴다.
