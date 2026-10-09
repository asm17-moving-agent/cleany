# 검증 및 외부 데이터 색인

코드·실행 계약·재사용 가능한 검증 도구는 Git으로 관리한다. 과거 실행 원본, 날짜별
분석 보고서, 발표 자료와 캠페인 전용 도구는 외부 데이터 루트에 보관한다.
기본값은 worktree 옆 `../cleany-data`, 변경 변수는 `CLEANY_DATA_ROOT`다.
이 호스트의 위치는 `/home/ehdrms/workspace/cleany/cleany-data`다.
이 데이터는 Git clone에 포함되지 않으므로 다른 호스트에는 필요한 fixture를 별도 전달한다.

| 자료 | 데이터 루트 아래 위치 |
|---|---|
| 현재 시나리오 입력 | `fixtures/cad-fortress-command-hold/` |
| 새 시나리오 실행 결과 | `runs/scenarios/<run-name>/` |
| 이전 artifacts 전체 | `runs/historical/20261010/artifacts/` |
| 삭제된 SCRUM-426 worktree 전체 압축본 | `archives/legacy-20261010/scrum-426-scenarios-20261010/worktree.tar.zst` |
| 이전 validation 보고서·navigation 분석·ChatGPT 문맥 | `reports/legacy-20261010/docs/` |
| 과거 캠페인·SLAM 실험 도구와 테스트 원본 | `experiments/legacy-20261010/` |
| 발표 디자인과 참조 이미지 | `presentation/legacy-20261010/` |
| worktree별 colcon 출력 | `builds/<worktree-name>/` |
| 이관 대응표·파일 해시·검증 로그 | `migrations/20261010-repository-cleanup/` |

`relocation-manifest.json`은 원래 경로 → 외부 경로, 각 파일 SHA-256/크기 또는
symlink 대상과 대조 결과를 담는다. 과거 로그·보고서·manifest의 절대 경로는 당시
증거를 보존하기 위해 수정하지 않았다. 옛 문서의 링크는 이 대응표와 원래 commit으로
찾는다. 보관한 실험 도구 역시 당시 소스 사본이며 현재 checkout에서 바로 실행하는
도구가 아니다. 필요하면 기록된 commit의 별도 checkout에 복원해서 재현한다.

현재 fixture는 과거 원본과 분리했다. `repo://`와 `fixture://`로 URI만 이식하고
transitive asset hash가 같은지 대조한 기록을 fixture의 `relocation-manifest.json`에 둔다.
기존 `profile-manifest.json`은 과거 provenance이며 새 위치의 검증 기록과 구분한다.

## 보존한 검증 범위

- 이전 SCRUM-426 연속 시험: 원래 6좌석 × 10회의 runtime SUCCESS 60/60,
  별도 ground-truth 오차 기준 57/60. fresh 시험과 합산하지 않는다.
- 2026-10-10 통합 후 기존 smoke: 한 인스턴스에서 44 → 16 왕복 2/2 SUCCESS,
  실측 RTF 1.818 / 1.847. 상세 원본은 historical의
  `scenarios/integration-20261010-command-hold-smoke-r3/`에 있다.
- 위 결과는 과거 소스·입력·실행 환경의 증거다. 데이터 이관 후 재검증 결과는
  새 run과 migration 검증 기록으로 구분한다. 시뮬레이션 주행이며 책상 작업은 mock이다.
  실제 로봇·팔·Backend 전체 E2E를 증명하지 않는다.

현재 실행법과 판정 계약은 [시나리오 도구](../../tools/scenario_validation/README.md),
[Workspace](../../ros2_ws/README.md), 각 패키지 README를 따른다.
KB submodule, runtime mesh/legacy 모델 선택, 안전 회귀 검사와 최종 command-hold profile은 유지한다.
