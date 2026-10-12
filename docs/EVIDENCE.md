# 실행 원본 색인

- 현행 사용법: [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md)
- Data root: ../cleany-data, 변경: CLEANY_DATA_ROOT
- 외부 fixture·asset: clone에 미포함. 별도 전달·hash 확인
- 과거 pass 수·RTF·결과: 현재 검증으로 재사용 금지

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

## 재현·보존

- 문서 보관 source: fbe5854. Commit·dirty·SHA-256: 보관 manifest
- 과거 경로 대응: relocation-manifest.json
- 절대 경로·원본 hash: 일괄 치환 금지
- 재현: 기록된 commit의 별도 checkout + 해당 환경·입력
- Fixture 현재 위치와 profile-manifest의 과거 provenance 구분
- 타 브랜치 자료: 해당 버전의 근거. 제품 결정은 KB 검토·Decision 절차
