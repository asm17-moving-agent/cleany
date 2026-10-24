# Nav2 개선·완화 재실험 비교 — 2026-10-13 KST

공통 목표 5개를 단계별로 각각 한 번 실행했다. 완화 최종은 네 컨트롤러 모두 Nav2 도착 5/5였고, 실제 GT 15cm/10° 관측은 DWB·MPPI·RPP 각각4/5, TEB5/5다. 이는 관측 건수이며 합격 기준·통계적 성공률·통합 점수는 없다.

## 단계별 관측

| 컨트롤러 | 원래 → 기존 제한 개선 → 완화 Nav2 도착 | 완화 GT 범위 관측 | 완화 5목표 합계 sim초 | 완화 평균 GT cm / ° |
|---|---|---|---|---|
| DWB | 5/5 → 5/5 → 5/5 | 4/5 | 119.61 | 12.66 / 2.35 |
| MPPI | 1/5 → 4/5 → 5/5 | 4/5 | 189.32 | 11.63 / 5.68 |
| TEB | 1/5 → 2/5 → 5/5 | 5/5 | 148.61 | 6.07 / 3.84 |
| RPP | 3/5 → 3/5 → 5/5 | 4/5 | 144.73 | 13.06 / 3.27 |

원래·보수 단계의 실패/timeout 종료 오차는 남은 거리이며 도착 정확도가 아니다. 실패를 포함한 총 소요시간으로 빠르다고 판단하지 않는다. DWB 역사적 baseline에는 의도된 속도 박스 위반이 발견되어 동일 조건 A/B로 간주하지 않는다. MPPI 최종 보수는 distant abort로 4/5이며, 이전 진행 메시지의 모두 도착 진술은 최종 기록으로 대체한다.

## 완화 목표별 정확도·시간

| 컨트롤러 | 목표 | sim초 | GT cm | GT ° | GT 15cm/10° 관측 |
|---|---|---|---|---|---|
| DWB | straight | 9.134 | 13.145 | 0.000 | 범위 내 |
| DWB | turn_arrival | 18.486 | 12.325 | 7.461 | 범위 내 |
| DWB | chair_gap | 20.832 | 16.364 | 0.000 | 범위 밖 |
| DWB | far_heading | 25.470 | 9.812 | 4.300 | 범위 내 |
| DWB | distant | 45.690 | 11.667 | 0.000 | 범위 내 |
| MPPI | straight | 12.168 | 12.585 | 0.279 | 범위 내 |
| MPPI | turn_arrival | 35.334 | 8.730 | 8.751 | 범위 내 |
| MPPI | chair_gap | 31.858 | 16.709 | 9.854 | 범위 밖 |
| MPPI | far_heading | 44.706 | 7.339 | 8.290 | 범위 내 |
| MPPI | distant | 65.252 | 12.783 | 1.241 | 범위 내 |
| TEB | straight | 9.718 | 7.316 | 7.594 | 범위 내 |
| TEB | turn_arrival | 22.902 | 5.836 | 1.586 | 범위 내 |
| TEB | chair_gap | 22.470 | 6.620 | 3.468 | 범위 내 |
| TEB | far_heading | 33.590 | 3.861 | 1.030 | 범위 내 |
| TEB | distant | 59.934 | 6.714 | 5.516 | 범위 내 |
| RPP | straight | 9.320 | 14.808 | 1.272 | 범위 내 |
| RPP | turn_arrival | 20.936 | 10.987 | 4.372 | 범위 내 |
| RPP | chair_gap | 33.390 | 16.021 | 4.377 | 범위 밖 |
| RPP | far_heading | 27.992 | 11.301 | 4.852 | 범위 내 |
| RPP | distant | 53.096 | 12.185 | 1.479 | 범위 내 |

의자 사이 목표 GT XY는 DWB16.36cm, MPPI16.71cm, RPP16.02cm로 남았다. Nav2는 추정 pose를 사용하며 마지막 GT/TF는 비동기 관측이다. 이 차이를 tolerance 확대나 재실행으로 숨기지 않았다. TEB의 이 목표 GT XY는6.62cm다.

## 연산량·부드러움

| 컨트롤러 / 호스트 | 주기 | 목표별 compute p95 범위 ms | 평균 선형 jerk RMS m/s³ | 평균 각 jerk RMS rad/s³ | joint 최대 rad/s |
|---|---|---|---|---|---|
| DWB / ARM64 | 20Hz | 11.895–14.025 | 0.387 | 0.350 | 4.9928 |
| MPPI / ARM64 | 20Hz | 10.213–25.522 | 0.337 | 0.233 | 3.9613 |
| TEB / x86 | 10Hz | 8.319–14.421 | 0.827 | 1.309 | 4.9900 |
| RPP / x86 | 20Hz | 0.126–0.354 | 0.596 | 1.419 | 4.8989 |

compute는 controller 내부 steady wall 시간이다. DWB·MPPI는 같은 ARM64, TEB·RPP는 같은 x86에서 측정했다. 서로 다른 호스트 절대 compute/CPU 순위를 만들지 않는다. RPP의 별도 Python wheel limiter 비용은 controller compute에 포함되지 않는다. CPU/RSS·median/p99·주기·RTF·예외·50ms 초과는 comparison.csv/JSON에 목표별로 보존했다. DWB RSS max와 나머지 RSS p95를 같은 지표로 합치지 않았다.

GT 속도20Hz sim-time 보간, centered5point smoothing, 양끝.25초 trim 후 jerk를 계산했다. 표는 각 목표 RMS의 동일 가중 산술 평균이며 pooled RMS가 아니다. 전체20목표가 도착한 완화 단계끼리만 이 요약을 제시했다. MPPI는 선형·각 jerk 평균이 낮게 관측됐으며 RPP·TEB는 방향 변화가 더 많았다. 속도·wheel slew·padding·주기가 달라 controller 고유 특성의 단독 증거는 아니다.

정지 정의: DWB/TEB/RPP는 병진속도<.01m/s라 제자리 회전을 포함하지만 MPPI 새 결과는 병진<.01 AND 각속도<.02다. 각 축 반전·motion starts/stops는 결과에 보존하되 정지 횟수는 정의가 달라 직접 순위화하지 않는다. 오래 정지한 실패의 낮은 jerk를 개선으로 해석하지 않는다.

## 설정 차이와 해석

- 공통: 휠 반경.0635m, wheelbase.35m, separation.6038m, 실제 footprint/물리 collision geometry, 지도·5목표·StraightFirstPlanner, goal checker15cm/10° 유지. 완화는 외부 pan/freshness/CollisionMonitor/voxel veto를 shadow 출력으로 분리하고 최종 wheel envelope를 적용했다. 내부 collision/feasibility 검사는 유지했다. 240sim초 소프트 표식, hard1200sim/1800wall초다. 실제 wheel command와 joint는 최종20건 모두5rad/s 이내다. 원시 TEB/RPP 명령은 limiter 앞에서5를 넘을 수 있어 최종 출력과 구분했다.
- DWB: rollout 안에서 coupled cap4.99 적용, 5×5×7 샘플과 목표 방향 탐색, 목표 회전 critic lookahead1초. full collision rollout2초 유지. 20Hz, native acceleration tuning, 추가 padding.02/inflation.8 유지. 별도 wheel slew는 쓰지 않았다.
- MPPI: 도착 XY 탐색 억제와 필터 후 제한 재적용, path orientation critic 조정 및 gamma tuning. optimized/sampled sequence와 최종 출력에 coupled cap 적용. 20Hz, 기존 footprint/costmap 유지. 별도8rad/s² wheel slew는 쓰지 않았다.
- TEB: min_obstacle_dist.05, homotopy OFF/single band, dt_ref.1, controller10Hz/출력20Hz. cap4.99 및 wheel slew8rad/s², actuator의 독립 축 제한 대신 coupled slew 적용. 확장 footprint/padding 유지.
- RPP: 곡률 보존 vx/wz 축소, lookahead 하한.18m, vx.30/wz.35, native collision TTC1초. cap4.80 및 wheel slew8rad/s², 추가 여유.08m와 costmap padding.02 제거. 실제 CAD footprint와 inflation.8 유지. controller/출력20Hz.

현재 결과는 각 controller에 맞춘 전체 프로파일 비교다. 속도 상승·가드 우회·wheel slew·footprint 여유·주기가 동시에 바뀌어 순수 controller 효과를 분리할 수 없다. 사용자 요청한5목표 각1회 재실험을 완료했으므로 추가 반복은 수행하지 않았다. 동일 조건의 엄격한 controller 비교가 필요하면 actuator/slew와 추가 footprint 여유를 통일한 별도 단계가 필요하다.

관측상 DWB는5목표 합계가 가장 짧았고, MPPI는 각속도 jerk가 낮았다. TEB는 이번20건 중 유일하게 모든 목표의 GT15cm/10° 범위에 들어왔다. RPP는 같은 x86의 TEB보다 native compute 비용이 매우 작았지만 각속도 jerk와 방향 반전이 컸다. 모든 판단은 이번 단일 실행과 각 설정에 한정된다.

## 접촉·미측정·운영 기록

완화 최종20건은 기록된 환경 접촉0 episode다. floor heartbeat와 collision 이름, DWB의 별도 body positive control로 센서 경로를 확인했다. contact sensor 주기/episode 정의는 다르며 zero 관측은 연속적인 무충돌 보장이나 물리 검증이 아니다. 보수 단계 접촉은 미계측으로 null이다. 연속 최소 실제 clearance와 penetration depth는 공통 미측정으로 null이며 별도 정적 XY 투영 간격을 실제 접촉·clearance로 대체하지 않았다.

TEB 초기 pilot joint5.302rad/s, RPP 무-slew pilot5.493rad/s 초과 기록은 진단 자료이며 최종 결과에서 제외됐다. 최종 cap/slew 적용 결과와 섞지 않았다. RPP 정리 중 기존 legacy launch와 그 하위 별도 friction Gazebo를 함께 종료한 실수가 보고됐다. 해당 당시 상태/경위는 RPP REPORT와 legacy_owned_processes.json에 보존됐다. 전체 물리 로봇 적용 및 git push는 없었다.

## 자료·재현

- comparison.json/CSV:60행(4controllers×3stages×5goals), null 이유와 원본 경로, source snapshots 및 SHA256. summary.csv:12단계 요약.
- build_comparison.py: 비교 생성 및20개 완화 결과의 상태/command·joint 상한/접촉 검증. source_snapshots: 읽어온 원격 TEB/RPP 보고서와 결과. 원시 bag은 각 원본 경로에 그대로 보존했다.
- 로컬 DWB 보고서: /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/REPORT.md
- 로컬 MPPI 보고서: /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/sol_resume_20261013/REPORT.md
- 원격 TEB 보고서: /home/donglee/workspace/cleany/controller-optimization-20261013/teb/REPORT.md
- 원격 RPP 보고서: /home/donglee/workspace/cleany/controller-optimization-20261013/rpp/REPORT.md
- 각 보고서에 최종 YAML/patch/reproduce/raw bag/log/runtime parameter checks가 연결되어 있다.

재생성: `python3 build_comparison.py`. 이 스크립트는 보존된 스냅샷을 읽으며 실험을 재실행하지 않는다.
