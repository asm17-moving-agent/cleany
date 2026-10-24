# 복구된 Nav2 실험 색인

## 마지막 검증 구성

2026-10-20 신규 실행: **MPPI `candidate_diverse` + Smac Lattice 공식 생성 primitive144**.
Omni, 지도 격자5cm, 곡선 최소 반경 설정10cm, 16headings.
8경로 각1회 Nav2 도착8/8, 총278.648sim초, 관측 비바닥·비자기 접촉0.
이전 50cm primitive 결과는260.812sim초였으며, 6/8번 시간 증가와 정지 증가가 있어
새 설정이 모든 지표에서 우수하다는 뜻은 아니다.

| 파일 | 용도 |
|---|---|
| `profiles/official-omni-r10cm.json` | 마지막 실행의 공식144개 프리미티브 원본 |
| `profiles/stock-omni-r50cm.json` | 이전 비교의 기본144개 |
| `profiles/previous-tuned-272.json` | 더 이전 수동 추가 튜닝272개 |
| `profiles/mppi-lattice-r10cm-benchmark.original.yaml` | 마지막 실행 전체 parameters 원본 (원격 절대경로 포함) |
| `profiles/benchmark-recovery.xml` | 같은 실행의 BT |
| `profiles/dwb-final-reference.yaml` | DWB 최종 비교 설정 |
| `profiles/theta-doorway-overrides.yaml` | Theta* 문 통과 개선 설정 |

프리미티브 SHA-256:
`01165b98a42f01c0676f9ee6e691a9ed27b72dc7413c18a3a92b830ca3f2756e`.
반경10cm는 Generator 입력이며 차체의 물리 최소반경 측정값이 아니다.

## 코드 및 설정 이력

- `snapshots/remote/doorway-diagnosis-20261014/desktop-experiment/`:
  MPPI 원본, candidate/candidate_v2/candidate_general/candidate_diverse,
  후보별 설정, 마지막 generalization/common.yaml, Gazebo runner와 분석 도구.
  현재 활성 구현은 `ros2_ws/src/cleany_navigation/mppi`에 통합했다.
  `candidate_diverse/src`의 알고리즘을 보존하고 namespace/플러그인 소유권만 변경했다.
- `snapshots/remote/controller-optimization-20261013/teb/`:
  ROS2 TEB 소스 (`sol61_20261013/teb_local_planner`), 튜닝 단계별 설정과 보고서.
- `snapshots/remote/controller-optimization-20261013/rpp-source/ros2_ws/src/cleany_navigation/`:
  curvature-limited RPP 구현/테스트. 같은 상위 디렉터리의 rpp/에 단계별 설정이 있다.
- `snapshots/local/controller-optimization-20261013/`: 로컬 DWB/MPPI 단계별 설정.
- `snapshots/mppi-local-20261013/`: 초기에 만든 MPPI 패치와 살아 있던 미커밋 변경.
- `snapshots/local/lattice-official-regeneration-20261016/`:
  공식 Generator 코드/입력, 원본·재생성 primitive, 정적 비교 도구와1번 실제 주행 기록.
- `snapshots/local/theta-mppi-doorway-20261016/`: Theta* 개선 patch/before 소스/검증.
- `snapshots/local/{planner-lattice-theta-20261016,controller-planner-matrix-20261020,mppi-lattice-official144-20261020}/`:
  8경로/4조합/10cm 재실험 runner, 설정, 보고서, 요약 CSV/JSON.
- `snapshots/remote/{facility-all-seats-lattice-static-20261016,facility-planners-static-20261016}/`:
  18F 좌석별 Lattice/NavFn/Smac2D/Hybrid-A*/Theta* 정적 탐색 실험.

스냅샷 전체는 `COLCON_IGNORE`로 활성 ROS 패키지 탐색에서 제외했다.
과거 runner는 원격 경로에 의존하며, 복구본은 실행 경로를 임의로 치환하지 않았다.
대용량 rosbag/시계열/이미지/영상/빌드 캐시는 이 Git 복구본에 중복 복사하지 않았다.
필요한 전체 결과는 `results/` 연결과 원본 데이터 경로에서 확인한다.

## 출처 및 검증

- `provenance/local-files.json`: 로컬 파일 경로/복사 위치/해시.
- `provenance/remote-files.json`: 원격 백업의 파일별 원본 해시와 경로.
- `provenance/recovery-audit.json`: 복구 기준 커밋, 선택 소스, 확인 범위/제약.
- `provenance/dwb-44c083a.patch`: 기준 커밋 대비 DWB 튜닝 patch.
- `provenance/validation.json`: 이번 테스트/빌드 결과.
- `INDEX.json`: 주요 버전의 기계 판독 색인.

원격 원본 백업:
`/home/donglee/workspace/cleany/cleany-data/archives/workspace-cleanup/20261017-182827/`.
선택 추출 압축본과 작업 로그:
`/home/ehdrms/workspace/cleany-data/recovery/nav2-tuning-20261024/`.
