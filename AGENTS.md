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

- KB와 코드 기능 구현이 불일치하면 해당 사안의 가장 최근 사용자 대화에 있는 명시적 지시와 확정된 합의를 우선한다.
- 관련 대화를 확인할 수 없거나 결정이 모호하면 사용자에게 확인한다. 미확정 내용을 추측해 확정하거나 하드코딩하지 않는다.

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
- 설정값은 코드에 숨겨 하드코딩하지 말고 패키지 `config/` 또는 ROS parameter로 이동할 수 있게 작성한다.
- ROS 패키지 dependency를 추가하면 `package.xml`과 `setup.py`/`setup.cfg`를 함께 갱신한다. 외부 의존성은 공식 rosdep 키를 사용하며, 공식 규칙으로 설치할 수 없는 경우에만 별도 설치 방법을 검토한다.
- 생성 파일, 대용량 asset, mesh 파일은 명시 요청 없이 재생성하거나 포맷하지 않는다.

### 주석과 Docstring

- Python은 Google-style Docstring, C++은 Doxygen을 사용한다. 코드의 주석과 Docstring은 간결하고 자연스러운 영어로 작성한다.
- 주석의 양보다 새로운 개발자가 역할, 설계 이유, 인터페이스 계약과 제약사항을 이해하는 데 도움이 되는지를 우선한다. 함수명, 변수명, 타입 힌트와 구현에서 자명한 동작을 반복하지 않는다.
- 단순한 함수는 Docstring을 생략하고, 설명이 필요하면 한 줄부터 시작한다. 핵심 알고리즘과 Public API는 필요한 단위, 좌표계, 반환 조건, 부작용과 예외를 설명한다. `Args`, `Returns`, `Raises`는 유의미한 추가 정보가 있을 때만 작성한다.
- ROS 콜백은 중요한 처리 조건과 예외 동작을 짧게 설명하고, Node 클래스에 적은 토픽과 메시지 정보를 반복하지 않는다. 함수 내부 주석은 비직관적인 구현 이유, 불변 조건과 제약사항에만 사용한다.
- 실제 구현에서 확인한 사실만 작성한다. 설계 의도와 동작을 추측하지 않고 Mock, Simulation과 실제 하드웨어를 구분한다. KB의 아키텍처 설명과 같은 정보를 여러 위치에 중복하지 않는다.
- 코드의 계약이 바뀌면 관련 주석도 함께 갱신하고, 잘못되거나 오래된 설명은 수정 또는 제거한다. 주석 작업을 이유로 동작, 공개 API, ROS 인터페이스를 바꾸거나 불필요한 리팩토링을 하지 않는다. 발견한 버그와 구조적 문제는 임의로 고치지 않고 별도로 보고한다.

ROS 2 Node 클래스는 역할을 먼저 설명하고, 다음 항목 중 실제로 사용하는 것만 구조화한다.

- `Subscriptions`, `Publishers`: 기본 토픽명, 메시지 타입과 입력 또는 출력의 의미.
- `Services`, `Actions`: 기본 이름, 타입, Client 또는 Server 역할과 요청의 의미.
- `TF`: 발행하는 Transform, 관련 좌표계와 다른 노드와의 책임 분리.
- `QoS`: 통신 호환성에 중요한 설정. 개별 endpoint 설명에 포함해도 된다.
- `Notes`: 초기화 조건, 예외 동작과 중요한 제약사항.

파라미터로 변경할 수 있는 이름은 기본값임을 표시한다. 아래 예시는 형식 참고용이며, 실제 이름, 타입과 동작은 작성 대상 코드에서 확인한다.

```python
class WheelOdometryNode(Node):
    """Estimate planar odometry from cumulative wheel angles.

    Subscriptions:
        joint_states (sensor_msgs/msg/JointState): All four wheel positions
            in radians; input_topic default. QoS: sensor_data.

    Publishers:
        wheel/odom (nav_msgs/msg/Odometry): Pose in odom and velocity in
            base_link; topic and frame names are parameter defaults.

    TF:
        Does not broadcast; the odometry relay owns odom -> base_link.

    Notes:
        The first sample, non-increasing time and velocity-window warm-up
        produce no message. Invalid wheel values raise a validation error.
    """
```

## 4. 검증 규칙

- 패키지 전용 자동 검사는 해당 패키지 `test/` 또는 `tests/`, 명시 실행 검사는 `runtime_test/`에 둔다. 루트 `tests/tools/`는 저장소 도구의 자동 검사, `tests/integration/`은 패키지를 연결하는 검사에 사용한다. 시나리오 실행, 관찰, 보고 코드는 `tools/validation/`에 두고 자체 단위 테스트와 분리한다.

- 코드를 바꿨으면 가장 작은 관련 테스트를 먼저 실행한다.
- FSM/core logic 변경: 해당 패키지 pytest를 우선 실행한다.
- ROS package manifest, launch, install data 변경: 관련 패키지 또는 workspace build를 실행한다.
- 여러 패키지 경계나 ROS interface를 바꿨으면 전체 workspace test까지 고려한다.
- 반복 빌드와 테스트는 루트 Makefile을 사용하고, 세부 옵션은 해당 launch의 `--show-args`와 Makefile을 따른다.
- 공통 빌드는 `docs/DEVELOPMENT_SETUP.md`, 실행은 `docs/RUNNING.md`, 검증은 `docs/TESTING.md`를 따른다.
- 검증을 실행하지 못했으면 최종 응답에 이유와 대신 확인한 내용을 적는다.

## 5. 대화 기반 plan과 KB 기록

plan은 실행 계획이고, Decision은 실제로 확정한 선택과 이유다. plan 승인이나 구현 성공만으로 미결정 정책을 확정하지 않는다.

1. 대화에서 목적, 변경 범위, 선택지, 검증 방법과 미결정 사항을 plan으로 정리하고 확정된 합의와 구분한다.
2. KB 작성이 요청되면 관련 대화 요약과 plan을 `40_RAW`에 보존한다.
3. 합의된 제품 기준은 `10_PLANNING`, 기술 기준은 `20_TECHNICAL`, 실제 결정은 `30_DECISIONS`에 반영한다. 실제로 제기한 미결정 판단은 해당 계층의 `99 - Questions.md`에 둔다.
4. Decision에는 선택, 논의된 대안, 확인된 이유, 영향과 결정 근거를 간결하게 기록한다. 근거나 수치를 만들지 않는다.
5. 작업 브랜치에서 KB 품질 검사를 실행하고 사람이 GitHub PR로 검토한다. 공유 기준은 검토와 병합이 끝난 문서를 따른다.

- Planning과 Technical의 `출처`에는 Raw 근거를, `관련 결정`에는 Decision을 연결한다. Decision의 `출처`에는 결정 근거를 표준 Markdown 링크로 연결한다.
- 결정 변경은 새 Decision과 `supersedes`, `superseded_by`로 기록하고 관련 기준 문서도 갱신한다.
- 실행 절차는 `docs/RUNNING.md`, 구현 계약은 패키지 `docs/`에 둔다. KB 편집은 명시 요청과 해당 submodule의 `AGENTS.md`를 따른다.

## 6. Jira와 GitHub

정보 소유권은 [KB 협업 안내](docs/cleany-docs/README.md#협업-도구와-구현-문서의-경계)를 따른다. Jira는 작업 상태, 담당자와 일정, GitHub PR은 검토와 승인 이력을 관리한다.

### 작업 분해

- 논리적 분해는 `Epic → Story → Task`다. Epic은 대분류, Story는 큰 기능·구현 단위, Task는 필요한 기술적 작업과 검증 항목이다.
- 실제 Jira에서는 Story와 독립 `Task` 모두 관련 Epic을 `parent`로 연결한다.
- 필요한 Task는 `Blocks`로 Story를 연결한다: `Task blocks Story` / `Story is blocked by Task`. 공통 Task는 필요한 Story들에 각각 연결한다.
- Task 간 선행 의존성도 `Blocks`로 연결한다. 생성·정리 시 유형, Epic `parent`와 링크 방향을 확인한다.
- Story 완료는 연결된 Task 상태와 자체 완료 조건을 함께 확인한다. Task 완료나 PR 상태만으로 Jira 상태를 자동 변경하지 않고 합의된 `Done` 기준을 따른다.

### Jira 작성

- 제목과 본문은 짧은 명사형 개조식으로 작성한다.
- 본문은 `목적 및 개요 / 완료 조건 / 결과`로 구성한다. 결과에는 진행 결과와 검증 근거를 요약한다.
- 선행 작업과 의존관계는 issue 링크로 관리하고 본문에 중복 기재하지 않는다.
- KB나 구현 README 본문은 복사하지 않는다. 관련 문서는 버전·작업 브랜치를 식별할 수 있는 링크로 연결하고 PR도 연결한다.

### PR과 보고

- PR에는 변경 범위, 검증 결과와 관련 Jira issue를 연결한다.
- CI 통과, 사람 승인, `main` 병합과 실물 검증을 구분해 보고한다. 병합 조건은 현재 브랜치 보호 규칙과 팀 운영 기준을 확인한다.
- commit, push, PR 생성과 병합은 사용자가 요청한 범위에서 수행한다.
