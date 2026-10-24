# DWB 보수적 기준 최종 결과 — 2026-10-13

원래 비교 source b3ca816ba420452ee7960837bc16ec74148e07b0에서 분리한 전용 checkout을 사용했다. local commit 538e9231ac68a4a95ebed4c7961d995ce539fc26, navigation tests 192개 통과. push 없음. 물리 로봇 실험 없음.

5개 목표를 고정 설정으로 각각 1회 실행했다. Nav2 status 4는 추정 pose에 대한 도착이며 GT 허용오차 보장을 뜻하지 않는다. 15cm/10도 goal checker, 240sim초/360wall초, RTF 목표 1, common planner/map/world/model/sensors/AMCL와 safety chain을 유지했다.

| 목표 | Nav2 status | 기준→최종 sim초 | 기준→최종 GT XY cm | 최종 GT yaw ° | compute median/p95/p99 ms |
|---|---:|---:|---:|---:|---:|
| straight | 4 | 29.43 → 19.91 | 12.72 → 12.30 | 0.55 | 3.77/10.83/12.69 |
| turn_arrival | 4 | 38.92 → 50.63 | 9.17 → 11.29 | 7.93 | 6.08/11.99/14.19 |
| chair_gap | 4 | 51.30 → 64.35 | 17.23 → 18.97 | 1.30 | 4.77/10.49/12.84 |
| far_heading | 4 | 57.93 → 57.77 | 9.54 → 7.93 | 8.32 | 6.56/13.52/15.63 |
| distant | 4 | 168.59 → 142.82 | 12.48 → 11.70 | 2.14 | 4.97/10.72/12.49 |

속도 상한 초과 측정에서 first command가 설정 box 밖으로 나오는 문제를 rollout 내부 component cap으로 수정했다. 목표 bearing과 기존 가속/감속 projection box가 교차할 때만 그 ray를 선택하는 옵션을 추가했다. 샘플 7×7×11→5×5×7, goal-directed radius .75→4.5m, translation acceleration/deceleration .4→.2m/s²를 선택했다. prediction horizon 2s와 0.05s discretization, 모든 footprint/path critics는 유지했다.

대표 실험은 pilot sampling(중단, 개선 증거 제외), sampling chair-gap 59.27s/GT17.13cm, radius 확장 chair-gap 52.49s/GT17.83cm, ray 이전 straight31.85s, ray straight18.87s, ray chair-gap56.19s/GT18.23cm, acceleration .2 straight17.09s 순으로 진행했다. 각 후보는 원인별 대표 구간 1회이며 AMCL 초기 추정과 scheduler가 달라 단일 관측을 통계적 인과/성공률로 해석하지 않는다.

기준 5회에도 command box 위반이 있었다. final /cmd_vel vx 최대 .18027756m/s 등이 의도된 .15 상한을 넘었다. 따라서 기준 시간은 동일 box가 보장된 엄격한 A/B 비교가 아니라 역사적 참고치이다. 기준 same-host compute profile straight는 median15.17/p9527.14/p9928.12ms, 14.79sim초였으며, 공식 straight29.43sim초와 별도이다.

| 목표 | RTF | command dt median/p95 s | CPU % 한 코어 / RSS MiB | joint wheel max rad/s | jerk RMS/p95 m/s³ | translation 정지초 / 정지횟수 |
|---|---:|---:|---:|---:|---:|---:|
| straight | 0.935 | 0.040/0.060 | 17.4/47.5 | 2.77005 | 0.891/1.781 | 2.85/8 |
| turn_arrival | 0.938 | 0.040/0.060 | 20.4/48.1 | 2.57591 | 0.652/1.163 | 17.45/27 |
| chair_gap | 0.935 | 0.040/0.060 | 19.7/48.2 | 3.59289 | 0.762/1.781 | 10.30/47 |
| far_heading | 0.927 | 0.040/0.060 | 22.5/48.3 | 2.91464 | 0.566/0.463 | 12.85/7 |
| distant | 0.933 | 0.040/0.060 | 18.9/48.8 | 4.27255 | 0.987/2.380 | 18.80/77 |

모든 관측 command 단계의 |vx|≤.15, |vy|≤.10, |wz|≤.14와 command/joint wheel≤5를 확인했다. compute 50ms 초과는 최종 5회 모두 0이다. compute 예외는 turn_arrival 7회, far_heading 1회이며 나머지는 0회다. 예외가 발생해도 해당 경로가 최종 도착한 것과 예외가 없었던 것은 별개이다. CPU는 같은 ARM64 호스트의 관측치이며 baseline 동시 부하와 다를 수 있다. 다른 호스트 CPU 절대값 비교는 하지 않는다.

jerk는 GT twist 20Hz sim-time 보간, centered5point smoothing, 두 gradient와 양쪽 .25s trimming이다. 정지초는 translation speed .01m/s 이하이며 heading rotation/정상 safety 대기도 포함한다. failed/timeout은 숨기지 않는 방식이며 이 최종 5회에는 timeout/abort가 없다. angular acceleration/jerk와 vx/vy/yaw 반전은 metrics.json에 있다.

chair-gap GT 오차는 미해결이다. 대표 ray chair-gap의 odom–GT 차이는 1.7mm, map–GT 차이는 3.73cm, 최종 map 목표 오차는14.62cm였다. SimpleGoalChecker가 15cm 안에서 종료하고 RotateToGoal이 translation을 멈추므로 controller의 fine candidate만으로 GT 중심까지 terminal correction을 계속할 수 없다. goal radius·sampling으로 GT15cm를 보장할 수 없으며, common localization/arrival policy의 별도 개선과 모든 controller에 대한 동일 적용이 필요하다. 본 단계에서는 common layer를 변경하지 않았다.

raw command receive sim/wall timestamp, pan reason, collision-monitor state, voxel decision, GT/map/odom snapshots를 보존했다. 주행 중 카메라 TURNING_PAN/WAIT_FRESH_DEPTH와 downstream stop이 관측됐다. stage 샘플은 비동기이며 단일 원인을 원자적으로 증명하지 않는다. startup의 WALL_TIMEOUT/Command timeout은 phase별 JSON으로 분리한다.

실제 static-world contact와 최소 true footprint clearance는 이 보수적 단계에서 계측하지 않았다(null). action success나 기본 action=0을 collision-free로 해석하지 않는다. fixture/map/world canonical geometry와 asset hashes 동일성은 common_fixture_verification.json, actual live params/library loading은 개별 run에 있다. 초기 pilot들은 고유 partition+domain225였고, ray chair-gap 이후 explicit discovery ports22551/22552를 추가했다.

재현 입력: ../final_patch.yaml, ../final.yaml, ../final.patch, ../REPRODUCE.md. 원시 bag/log, runner snapshot, source/library hashes와 metrics.json/metrics.csv는 이 directory에 있다. 원래 HANDOFF 및 중단 파일은 보존했고 infrastructure-only git provenance 오류는 resume61_straight_03에 남겼다. 자기 partition 프로세스는 모두 종료됐고 bag SQLite integrity를 확인했다. 이후 승인된 완화 단계는 별도 directory에 기록한다.
