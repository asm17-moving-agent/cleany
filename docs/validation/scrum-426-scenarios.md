# SCRUM-426 Gazebo 시나리오 반복 검증 결과

2026-10-09. 합의한 병렬 한계 측정, 전체 좌석 조사, 대표 fresh/continuous 반복,
이상 상황 반복을 모두 실행했다. **검증 실행 완료이며 모든 경로의 성공을 의미하지 않는다.**
원래 정상 실패와 기대 판정 미통과를 보존했다. 물리 로봇 E2E는 검증하지 않았다.

## 실행 환경과 의자 조건

- 별도 branch/worktree `test/scrum-426-scenarios`; local root
  `/home/ehdrms/workspace/cleany/scrum-426-scenarios`, desktop root
  `/home/donglee/workspace/cleany/scrum-426-scenarios`.
- desktop: Ryzen 5 9600X / RTX 5070, Ubuntu 22.04 ROS Humble/Fortress container,
  NVIDIA Ogre2 EGL. laptop: aarch64 `a16-fedora`, freedreno/Adreno X2-90 Ogre2.
- 각 worker의 ROS domain, Gazebo partition, DB, 결과/로그를 분리했다.
  요청 RTF 4, physics step 0.0066667초, compact static scene, reactive safety.
- 82개 좌석과 82개 의자 대응 검사. 원래 누락된 43/13 의자를 복원한 뒤
  **대상 의자 하나만 제거**한다. 모든 다른 모델을 보존한 82개 월드 검사 통과.
- continuous는 이전 의자를 복원하고 다음 의자를 제거한다. 실제 scene/info 원문
  60개를 독립적으로 파싱하고 hash를 대조했다. 모두 81개 의자, 해당 대상 하나만 부재.
- navigation/sensors는 Gazebo 실측, perception/planning/manipulation은 mock.
  팔 제외 simulation profile이며 실제 로봇·팔·물리 e-stop·D435 분리 증거가 아니다.

## 데스크탑 병렬 한계와 처리량

RTF는 미션 `/clock` 증가량 / wall time이다. 처리량은 준비/종료를 포함한 batch
시간 기준 성공 왕복/시간이다. 두 값을 합하거나 같은 지표로 해석하지 않는다.

| worker 수 | 긴 13번 왕복의 실측 RTF | 검증 결과 |
| --- | --- | --- |
| 1 | 2.145 | SUCCESS; 짧은 43번은 2.153 |
| 2 | 1.636–1.709 | 긴 왕복 3회 측정 모두 SUCCESS |
| 3 | 1.192–1.298 | 긴 왕복 3회 측정 모두 SUCCESS |
| 4 | 0.782–0.814 | 3회 × 4개 = 12/12 SUCCESS |
| 5 | 0.484–0.513 | 3회 × 5개 = 15/15 TIMEOUT |
| 6 | 0.335–0.343 | 6/6 TIMEOUT |

현재 profile/deadline과 13/43 기준 경로에서는 **왕복 성공 한계 4개**, 실측 2×는
단일 worker에서 확인됐다. 전체 좌석의 성공 한도나 하드웨어의 일반 한도로 주장하지 않는다.
5개에서는 짧은 43번이 성공해도 긴 13번이 실패하므로 짧은 시험만으로 한도를 정하지 않았다.

긴 왕복의 2/3-worker 처리량 3회 중앙값은 63.65/73.25 성공 왕복/시간이다.
전체 좌석 조사에는 3-worker를 선택했다. 단일 worker는 38.66이었다.
4/5-worker CPU 평균은 약 89–92%/96–98%, GPU 최대는 16% 이내,
5-worker MemAvailable은 약 31 GiB 이상이었다. CPU 포화와 속도 저하가 함께 나타났지만
독립 profiler로 각 계산·대역폭 병목을 분리한 결과는 아니다.

첫 2-worker configure 응답 누락 사례는 중단 로그로 보존했다. harness는 discovery 후
명시적인 lifecycle STARTUP을 사용한다. 이 준비 문제를 미션 성공으로 집계하지 않았다.

## 전체 좌석과 대표 반복

| 범위 | 실행 수 | 결과 |
| --- | --- | --- |
| baseline 전체 좌석 조사 | 82 | 27 SUCCESS / 55 FAILED |
| baseline 실패 55개 수정 후 조사 | 55 | 17/22 두 좌석 추가 SUCCESS; 조사 단계 latest 29/82 |
| 대표 6개 fresh × 10 | 60 | 44 SUCCESS / 16 FAILED |
| 한 인스턴스 6-seat sequence × 10 | 60 | 5 SUCCESS / 55 FAILED |

추가로 01 단독 및 13/43 대조를 수행해 correction queue는 총 58 missions다.
대표는 조사 단계에서 측정한 near/far/narrow mapped path/large turn/arrival error/
guard intervention 역할의 44/16/17/20/09/35다. 반복 실패 때문에 대표를 교체하지 않았다.

| 좌석 | fresh SUCCESS / 10 | continuous SUCCESS / 10 |
| --- | --- | --- |
| 44 | 10 | 2 |
| 16 | 10 | 2 |
| 17 | 2 | 1 |
| 20 | 2 | 0 |
| 09 | 10 | 0 |
| 35 | 10 | 0 |

continuous 첫 순환은 44/16만 성공, 두 번째는 44/16/17 성공, 3–10번째는 모두 실패했다.
실패 후 현재 위치·localization·obstacle memory를 유지하고 다음 미션을 실행했다.
teleport, 지도/기억 초기화, 안전 기준 완화로 통과시키지 않았다. fresh와 continuous는
worker 수와 scene 유지 방식도 달라 직접적인 성능 인과 비교로 해석하지 않는다.

전체 82개, correction 58개, fresh 60개와 continuous 60개의 실제 terminal mission ID와
좌석을 각각 확인했다. 모든 실행의 최종 cmd/GT 정지 및 launch 종료 검사도 통과했다.
continuous의 기대 정상 SUCCESS 판정은 **미통과**이며 실행 완료와 구분한다.

## 이상 상황 검증

두 호스트에서 7종 smoke를 각각 수행했고 각각 기대 판정 6/7이 통과했다.
노트북의 독립 fresh 5회 반복은 입력 35개와 terminal status 35개가 정확히 일치한다.

| 조건 | 반복 | 기대 판정 통과 |
| --- | --- | --- |
| outbound cancel | 5 | 5 |
| return cancel | 5 | 5 |
| synthetic safety stop | 5 | 5 |
| depth-clearance adapter loss | 5 | 0 |
| inserted obstacle | 5 | 5 |
| planner process loss | 5 | 5 |
| blocked target | 5 | 5 |

총 30/35 통과. 센서 안전 입력 유실 5회 모두 STALE_DEPTH와 실제 정지
(0.404–0.419 wall seconds)를 관찰했지만 cleanup cancel 후 return 실패로 terminal
FAILED였다. 기대 CANCELLED 판정은 통과하지 않았고 기대값을 변경하지 않았다.
raw depth camera는 계속 동작했으므로 물리 센서 분리 시험이 아니다.
장애물 clearance는 sampled GT/configured padded XY body 거리이며 접촉 센서 증거가 아니다.

## 실패 분석·개선과 검증 한계

- endpoint 회전의 한 구간 conservative bound가 과도하게 넓어 안전한 half-turn을
  거부했다. 회전을 작은 연속 검사 구간으로 나누고 같은 full-cell SAT/arc 안전 판정을
  유지했다(`3d6604d`). 저장 01 지도는 경로를 반환했지만 실제 01 왕복은 실패했다.
- 고정 body heading으로 통과하지 못하는 room 경로에 32방향 sparse body-heading
  탐색을 추가했다(`5d5c01b`). 기존 공유 2초 예산 안에서 full footprint/edge/live 검사한다.
  저장 지도에서는 성공했으나 실제 Gazebo controller progress 실패는 남았다.
- DWB 정지와 전진의 기존 점수가 같던 수집 사례를 확인하고 연속 path progress
  critic을 추가했다(`cabd0ea`). 기존 장애물·속도·도착·freshness/timeout guard는 유지했다.
  reactive mode에서는 pan gate가 observation_only이므로 이전 혼합 명령 차단 설명은
  이 모드의 원인 근거가 아니다. 수집 snapshot은 atomic controller replay가 아니다.
- 독립 laptop v5 overlay: **175 tests, 0 errors/failures/skipped**, plugin package build 완료.
  13/43 두 왕복 SUCCESS, 실측 RTF 1.883/1.701, 정지와 종료 검사 통과.
  room-a4-01/17/20 3개 병렬은 모두 FAILED(RTF 1.320/1.325/1.347).
  20은 target navigation OK와 mock 작업 후 return 실패였다. 전체 문제 해결로 주장하지 않는다.
- v5 20번 return 저장 지도는 start/goal footprint가 각각 안전했지만 32방향 탐색이
  129,456 states를 0.473초에 소진해 연결되지 않았다. 이는 이산 탐색 실패이며
  연속 공간의 실제 통행 불가능성 증명은 아니다.
- continuous 후반 44번 실패 snapshot을 size/FNV 검증 후 별도 보존했다.
  start footprint는 안전했지만 goal footprint는 거부됐다. search/live 지도의 lethal
  셀 6개가 goal footprint와 겹쳤으며 원래 정적 PGM에서는 해당 영역이 free였다.
  같은 지도의 32방향 탐색도 2,618,046 states/13.033초를 소진해 실패했다(timeout=false).
  이 진단 예산을 runtime에 적용하지 않았다. 해당 후반 관찰의 GT/map pose 차이는
  약 0.25m였지만 이것만으로 lethal 셀의 센서/TF 원인이나 실제 충돌을 확정하지 않는다.

8cm runtime arrival, 180초 navigation timeout, freshness, 속도와 정지 기준을 유지했다.
정상 왕복의 17/20 불안정, 지속 scene의 관측 지도/위치추정 문제와 depth 입력 유실 시
취소·복귀 terminal 정책은 후속 개선 항목이다. 모두 성공으로 완료 처리하지 않았다.

## 원본 증거와 종료 감사

`artifacts/scenarios/`에 fixture, 입력/월드/tool/binary hash, trace, result, shutdown,
resource samples, scenario queue/status, 실제 scene/info 원문과 실패 map bytes를 보존했다.
호스트/architecture/install prefix와 binary hash로 desktop v2와 laptop v5를 구분한다.
baseline 구현 `076b893`, desktop 반복 source `f9909ca`, local navigation 최종 변경
`cabd0ea`이다. 기존 branch/worktree의 사용자 변경은 건드리지 않았다.

- `plan_completion_audit.json`: scope별 queue와 미션 판정.
- `final_continuous_verdict.json`, `final_chair_scene_audit.json`: 60회 결과와 원문 의자 검증.
- `final_remote_manifest.json`, `final_sync_audit.json`: desktop 원본 5,723개,
  2,555,038,639 bytes를 local에 복사하고 전 파일 SHA-256 일치 확인.
- `final_remote_process_audit.json`, `final_local_process_audit.json`: desktop/local
  자식 기록 816/180개 확인, 이 작업의 남은 launch/worker/observer 없음.
- `planner-replay-continuous-late/`: 후반 실패의 immutable map과 별도 geometry/cell/SE2 재생.
- `laptop_repeat_verdicts.json`, `capacity_report.json`, 각 queue의 `*.status.json`.

Jira에 메시지를 보내거나 상태를 변경하지 않았다. 문서/로컬 commit과 원본 증거를 남겼다.
과정·중간 실패·추가 진단의 상세 기록은 [단계별 조사 기록](scrum-426-scenarios-history.md),
실행 도구와 mode 구분은 [검증 도구 README](../../tools/scenario_validation/README.md)를 따른다.

## 재실행

같은 source/version과 tested install prefix를 선택하고 새 이름/domain을 사용한다.
실행 중인 install을 덮어 빌드하지 않는다. fixture는 기존 검증 입력을 사용한다.
노트북 v5 대조 왕복 재실행 예:

```bash
python3 tools/scenario_validation/parallel.py \
  --root "$PWD" --container ros2-humble --count 2 \
  --seats seat-13,seat-43 --name laptop-reference-recheck \
  --runtime-overlay "$PWD/ros2_ws/install-scenario-runtime/setup.bash" \
  --navigation-overlay "$PWD/ros2_ws/install-navigation-v5/setup.bash" \
  --domain-base 160
```

대표 fresh 반복은 `representative_fresh_jobs.json`, continuous 순서는
`representative_continuous_jobs.json`, 이상 상황 35개 입력은
`laptop_scenario_repeat_jobs.json`에 있다. 각 job에 새 고유 이름을 부여해
`queue.py --root "$PWD" --jobs INPUT.json`으로 재실행한다. 기존 결과 디렉터리는 덮어쓰지 않는다.
