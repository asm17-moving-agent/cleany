# 실행 원본과 과거 문서 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md)을 따른다.
이 색인은 과거 특정 commit·입력·환경의 기록을 찾기 위한 것이다.
과거 pass 수·RTF·캠페인 결과를 현재 checkout의 검증 결과로 재사용하지 않는다.

데이터 기본 루트는 worktree 옆 `../cleany-data/`, 변경 변수는 `CLEANY_DATA_ROOT`다.
clone에는 외부 데이터가 포함되지 않는다. 필요한 fixture·asset은 별도로 전달하고 hash를 확인한다.

| 데이터 루트 아래 경로 | 내용 |
|---|---|
| `fixtures/cad-fortress-command-hold/` | 시나리오 기본 입력 |
| `fixtures/cad-fortress-command-hold-portable/` | local mesh/texture를 고정한 별도 입력 |
| `runs/scenarios/<run-name>/` | 새 scenario 결과·journal·로그 |
| `runs/historical/20261010/artifacts/` | 이전 실행 원본 |
| `reports/legacy-20261010/docs/` | 과거 검증·navigation 분석·ChatGPT 인수인계 사본 |
| `experiments/legacy-20261010/` | 과거 캠페인·SLAM 코드 사본 |
| `experiments/legacy-runtime-cleanup-b3ca816/manifest.json` | 레거시 코드의 commit/hash |
| `migrations/20261010-repository-cleanup/` | 이관 대응표·hash·검증 로그 |
| `archives/legacy-20261010/scrum-426-scenarios-20261010/worktree.tar.zst` | 이전 scenario worktree |
| `presentation/legacy-20261010/` | 발표 참조·원본 자료 |
| `builds/<worktree-name>/` | 해당 worktree의 colcon 출력 |
| `reports/documentation-reorganization-20261013/before/` | 이번 문서 개편 전 Markdown 사본 |
| `reports/documentation-reorganization-20261013/manifest.json` | 사본 source commit·dirty 상태·각 파일 SHA-256 |

이번 문서 사본의 source commit은 `fbe5854`다. 작업 전에 삭제돼 있던 검증 색인은
이 commit의 `docs/validation/README.md`로 복원한 사본이며 원래 작업 트리 삭제는 유지했다.
사본의 오래된 command·경로를 현행 사용법으로 실행하지 않는다.

`relocation-manifest.json`으로 옛 경로와 외부 경로를 대조한다. 과거 기록의 절대 경로·hash는
증거이므로 일괄 치환하지 않는다. 재현이 필요하면 기록된 commit의 별도 checkout을 사용한다.
현재 fixture의 새 위치 검증과 기존 `profile-manifest.json`의 과거 provenance는 구분한다.

다른 작업 브랜치의 설계·평가·ChatGPT 전달 자료는 해당 브랜치의 사본이다.
유효한 연동 계약만 구현 문서에 반영하고 고유 측정 결과는 provenance와 함께 보존한다.
제품·기술 결정이 되려면 KB의 검토·Decision 절차를 따른다.
