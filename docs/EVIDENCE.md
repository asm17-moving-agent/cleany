# 실행 원본 위치

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md)
가이드를 따릅니다. 실행 결과와 벤치마크 보고서는 Git에 포함하지 않습니다.

기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다.
과거 실행 색인과 날짜별 성능, 수거 시험 보고서 및 검증 JSON은
`CLEANY_DATA_ROOT/reports/evidence/`에 원래 상대 경로대로 보관합니다.
전체 과거 색인은 해당 디렉터리의 `docs/EVIDENCE.md`에서 확인합니다.

새 실행 원본은 `CLEANY_DATA_ROOT/runs/` 또는 `CLEANY_DATA_ROOT/reports/`에 두고
commit, 환경, 입력, 실패와 원본 위치를 함께 남깁니다. Clone에 포함되지 않은
fixture와 asset은 별도로 전달받고 hash를 확인합니다.

현재 충돌 구의 certificate와 개별 부품 포함 검사 원본은
`CLEANY_DATA_ROOT/reports/commit-validation-fixes-20261030/`에 보관합니다.
`certificates.npz`는 asset의 `coverage_certificate_sha256`으로 확인하며,
입력 URDF, 변경 전후 구, 실행 환경과 결과는 같은 디렉터리의
`current.urdf`, `head-repair.json`, `validation.json`에서 확인합니다.
