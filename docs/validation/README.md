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

## 2026-10-10 이관 후 확인

코드 기준 `b863202`: 외부 경로에서 전체 17개 패키지 build, colcon 집계 803건 중
실패/오류 0건·23건 skip을 확인했다(집계에는 CTest wrapper 포함).
초기 hand-eye 2건은 container Git 2.34의 `relativeWorktrees` 미지원으로 실패했고,
외부 cache의 Git 2.55를 해당 검사 PATH에만 지정해 관련 274건을 재검사해 통과했다.
시나리오 도구 48건, 빌드 경로·정리 도구 9건도 통과했다.

`runs/scenarios/external-data-20261010-smoke/`에서 같은 인스턴스로 44 → 16 왕복
2/2 SUCCESS, 실측 RTF 1.895 / 1.819를 기록했다. 목적지 오차 3.16 / 1.04 cm,
home 오차 3.77 / 5.72 cm이며 기존 8 cm·0.15 rad 기준을 통과했다.
원래 설정 배속 4와 안전/freshness/deadline 기준은 유지했다.
미션 journal 2건, 입력/자산 hash, 실제 적용 parameter, 최종 정지 및 자식 30개
정상 종료를 `migrations/20261010-repository-cleanup/gazebo-audit.json`에서 대조했다.
이 결과는 이관 smoke이며 60회 반복 시험을 다시 수행한 결과는 아니다.

Git에서 제외한 51개 파일은 `tracked-removals-verification.json`에 원래 commit과
외부 사본의 동일 hash를 기록했다. 기존 build/install과 웹 의존성 cache는 정리했고,
이전 build 로그는 `runs/build-logs/20261010/`, UI 시험 결과는
`runs/ui-tests/legacy-20261010/`에 보존했다. 현재 monitor 설치에 필요한 `web/dist`
생성물은 유지한다. 이관 후 컨테이너 안에서 새 overlay의 링크와 import도 확인했다.


## 레거시 runtime 코드 정리 원본

`experiments/legacy-runtime-cleanup-b3ca816/manifest.json`은 정리 전 Gazebo 패키지와
관련 테스트의 원래 commit/파일 hash를 보존한다. 재생·진단 도구는 이 사본에서
기록된 commit 기준으로 복원한다. 현행 센서 설정 함수와 안전/주행 코드는 저장소에 둔다.
