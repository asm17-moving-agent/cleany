# AGENTS.md

Cleany 로봇 엣지 시스템 구현 레포에서 AI 코딩 에이전트가 따라야 할 규칙이다.

## 1. 저장소와 환경

- 핵심 구현은 `ros2_ws/`의 ROS 2 workspace에 둔다.
- 기준 환경은 Ubuntu 22.04 VM의 native ROS 2 Humble이다. 설치와 system dependency는 `docs/DEVELOPMENT_SETUP.md`를 따른다.
- 기획 KB는 `docs/cleany-docs/` submodule이다. 비어 있거나 누락되었으면 초기화한다.

```bash
git submodule update --init --recursive docs/cleany-docs
```

- `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`, `__pycache__/`, `.pytest_cache/`, `.ruff_cache/` 같은 생성물은 편집하지 않는다.

## 2. 판단 기준과 문서

- KB와 코드 기능 구현이 불일치하면 해당 사안의 가장 최근 사용자 대화에 있는 명시적 지시와 확정된 합의를 우선한다.
- 관련 대화를 확인할 수 없거나 결정이 모호하면 사용자에게 확인한다. 미확정 내용을 추측해 확정하거나 하드코딩하지 않는다.
- 최신 요구사항과 현재 구현 상태를 구분한다. 구현 사실은 코드와 검증 결과로 확인하고 필요한 변경을 명시한다.
- 제품 범위, 아키텍처, 안전, 하드웨어와 런타임 근거는 KB를 참고한다. 참조와 작성 절차는 `docs/cleany-docs/AGENTS.md`와 그 안내 문서를 따른다.
- 구현 사실, ROS 인터페이스, 설정, 실행과 검증 방법은 관련 패키지 `README.md`에서 관리하고 코드 변경과 함께 갱신한다.
- 확인이 필요한 불확실성은 코드에 최소한의 TODO 또는 주석으로 남긴다.

## 3. 코드 작성

- Python은 type hint, 작은 dataclass/model과 명확한 port/interface를 선호한다.
- ROS 2 node와 순수 core logic을 가능하면 분리해 core는 pytest로 검증한다.
- 패키지 간 의존은 명시적 interface/message/config로 연결한다.
- 설정은 하드코딩하지 않고 `configs/mission/`, `configs/robot/` 또는 ROS parameter로 관리할 수 있게 작성한다.
- ROS dependency 추가 시 `package.xml`, `setup.py`/`setup.cfg`, 필요 시 `ros2_ws/rosdep/cleany.yaml`을 함께 갱신한다.
- 생성 파일, 대용량 asset과 mesh는 명시 요청 없이 재생성하거나 포맷하지 않는다.

## 4. 검증

- 가장 작은 관련 테스트부터 실행한다. FSM/core 변경은 해당 패키지 pytest를 우선한다.
- manifest, launch와 install data 변경은 관련 build를 실행한다. 패키지 경계나 ROS interface 변경은 전체 workspace test도 고려한다.
- 반복 빌드·테스트는 루트 Makefile을 사용한다. 공통 명령과 세부 native 명령은 `ros2_ws/README.md`, 패키지별 명령은 해당 README를 따른다.
- 검증을 실행하지 못했으면 이유와 대신 확인한 내용을 보고한다.

## 5. 대화 기반 plan과 KB 기록

plan은 실행 계획이고, Decision은 실제로 확정한 선택과 이유다. plan 승인이나 구현 성공만으로 미결정 정책을 확정하지 않는다.

1. 대화에서 목적, 변경 범위, 선택지, 검증 방법과 미결정 사항을 plan으로 정리하고 확정된 합의와 구분한다.
2. KB 작성이 요청되면 관련 대화 요약과 plan을 `40_RAW`에 보존한다.
3. 합의된 제품 기준은 `10_PLANNING`, 기술 기준은 `20_TECHNICAL`, 실제 결정은 `30_DECISIONS`에 반영한다. 실제로 제기한 미결정 판단은 해당 계층의 `99 - Questions.md`에 둔다.
4. Decision에는 선택, 논의된 대안, 확인된 이유, 영향과 결정 근거를 간결하게 기록한다. 근거나 수치를 만들지 않는다.
5. 작업 브랜치에서 KB 품질 검사를 실행하고 사람이 GitHub PR로 검토한다. 공유 기준은 검토와 병합이 끝난 문서를 따른다.

- Planning과 Technical의 `출처`에는 Raw 근거를, `관련 결정`에는 Decision을 연결한다. Decision의 `출처`에는 결정 근거를 표준 Markdown 링크로 연결한다.
- 결정 변경은 새 Decision과 `supersedes`, `superseded_by`로 기록하고 관련 기준 문서도 갱신한다.
- 구현 세부사항과 실행 명령은 구현 README에 둔다. KB 편집은 명시 요청과 해당 submodule의 `AGENTS.md`를 따른다.

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
