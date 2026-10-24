# 수동 경유점 없는 MPPI 공통 8경로 검증

생성: 2026-10-14T20:34:00

상태: 완료. 기록된 8개 중 Nav2 성공과 GT 30cm/10°를 동시에 만족한 실행은 7개.

## 적용한 설정

모든 목표에서 수동 사전 정렬, 중간 NavigateToPose 목표, M1 전용 설정, 문 구간 전용 경로 유지 BT를 제거했다. 원래 HOME에서 잠금 파일의 최종 목표 하나만 전달한다. 목표마다 새 월드와 독립 ROS graph를 사용했다. 목표 의자 하나만 제거하며 지도·footprint·최종 허용 오차는 이전 시험과 동일하다.

채택한 MPPI 소스는 기존 candidate_v2다. 1000개 중 128개 경로 SE(2) 후보(속도 8종 반복), PathAlign 가중치 12, weighted mean/filter 후 native footprint 출력 검증, 0.08m/12초 진행량 제한을 유지했다. 주기적 Smac 재계획은 성공한 결과만 현재 경로에 반영하며, 실패한 재계획이 기존 경로를 지우지 않는다. 동일한 bounded BackUp/재계획 BT를 모든 경로에 사용했다. 이번에 새로운 MPPI 비용 함수를 채택한 것은 아니다.

`common.yaml`이 8개 모두의 설정이며 `general_audit.json`은 실제 로드된 설정·라이브러리 해시, 목표 하나, 경유점 없음, 공통 BT와 물리 제약을 검사한다. 과거 M1 수동 경유점 성공을 이번 결과에 섞지 않았다.

## 8경로 결과

성공 조건: Nav2 status=4 **그리고** GT 위치≤0.30m, 방향≤10°. 휠 명령과 Gazebo joint 속도는 별도로 기록한다. 접촉 0은 계측된 비바닥·비자체 접촉이 없다는 뜻이며 독립적인 비침투 증명은 아니다.

| 경로 | 성공 | GT 위치 m | GT 방향 ° | sim s | wall s | joint 최대 rad/s | 비바닥 접촉 | compute p95 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| [straight](regression8/0_straight/metrics.json) | 통과 | 0.268 | 4.78 | 6.08 | 13.90 | 3.906 | 0 | 6.80 |
| [turn_arrival](regression8/1_turn_arrival/metrics.json) | 통과 | 0.040 | 7.96 | 16.29 | 38.20 | 4.456 | 0 | 4.57 |
| [chair_gap](regression8/2_chair_gap/metrics.json) | 통과 | 0.048 | 5.70 | 31.37 | 72.70 | 4.884 | 0 | 4.28 |
| [far_heading](regression8/3_far_heading/metrics.json) | 통과 | 0.015 | 7.30 | 31.34 | 75.10 | 4.443 | 0 | 4.57 |
| [distant](regression8/4_distant/metrics.json) | 통과 | 0.077 | 5.94 | 63.68 | 152.40 | 4.917 | 0 | 4.32 |
| [lateral](regression8/5_lateral/metrics.json) | 통과 | 0.299 | 0.14 | 2.77 | 6.50 | 3.886 | 0 | 4.62 |
| [narrow_seat](regression8/6_narrow_seat/metrics.json) | 통과 | 0.072 | 7.96 | 16.08 | 37.10 | 4.134 | 0 | 5.76 |
| [other_room](regression8/7_other_room/metrics.json) | 실패 | 1.162 | 59.50 | 100.77 | 244.90 | 4.932 | 0 | 8.75 |

## 비교 실험과 채택 판단

본시험 앞에 수동 경유점 없는 M1 대조와 새 비용 평가를 각각 실행했다. 두 실행 모두 원래 목표 하나만 전달했고, 다른 실행의 결과를 8경로 본시험에 대체하지 않았다.

- 기존 범용 수정: Nav2 4, GT 0.030m / 8.36°, 117.28sim초. [원본 기록](control_m1/metrics.json).
- SE(2) 구간 투영 비용 실험: Nav2 6, GT 1.142m / 59.78°, 115.40sim초. [원본 기록](segment_m1/metrics.json).

새 비용 평가는 후보의 위치·방향을 연속 경로 구간에 투영하여 비교하도록 구현했으며, 회전·이동 결합, 순수 회전, ±π 경계, 중복 점 단위 검사를 통과했다. 그러나 실제 M1에서 유효 출력 궤적 부족으로 복구 후 실패했다. 계산 비용도 증가해 채택하지 않았다. 소스는 candidate_general에 보존했다. 후보 생성은 바꾸지 않은 비교이지만, 실행마다 계획·센서·난수 조건이 달라 엄밀한 인과적 ablation이나 통계적 우위라고 해석하지 않는다.

저장된 원래 문 앞 상태의 native MPPI 재생은 gaussian / 경로 후보 / 새 구간 비용을 비교했다. 경로 후보 설정에서 이동·회전 결합 출력이 관찰되었다. 고정 위치·속도 0의 진단이며 폐루프 주행이 아니다. 각 프로세스의 Gaussian batch가 달라 정확히 같은 난수로 짝지은 비교도 아니다. [재생 요약](replay/summary.json).

## 한계와 보존 조건

각 경로 1회 검사다. 모든 결과가 통과하더라도 반복 성공률이나 미지의 문에 대한 범용성을 증명하지 않는다. 추가 5회 반복은 하지 않았다. 정체/실패한 실행의 짧은 시간이나 낮은 jerk를 개선으로 취급하지 않는다. 명령 wheel projection은 ≤5rad/s지만 물리 joint 순간값은 별도 지표다. GT는 평가에만 쓰며 주행 중 AMCL 위치를 GT로 주입하지 않았다.

원래 footprint와 padding=0, wheel 반경 0.0635m·wheelbase 0.35m·separation 0.6038m, Smac Lattice 5cm/0.5m omni primitive·reverse=true·smooth=false·max_iterations=-1·planning 80초, AMCL 0.01m/0.01rad·매 1sim초 nomotion update를 유지했다. 노트북/실물 로봇 프로세스와 기존 실험 결과는 변경하지 않았다. git push는 하지 않았다.

## 재현

```bash
cd /home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment
distrobox enter cleany-ros-humble-cdi -- python3 run_campaign.py \
  --profile general --install candidate_v2/install --bt "$PWD/recovery_keep_path.xml" --output "$PWD/generalization/reproduce8" --domain-base 220
```

이미 존재하는 실행 디렉터리는 덮어쓰지 않는다. 재현 시 다른 실행과 ROS domain이 겹치지 않아야 한다.

- [전체 CSV](regression8/summary.csv), [전체 JSON](regression8/summary.json)
- [공통 설정](common.yaml), [검증 결과](general_audit.json)
- 각 실행의 result.json, metrics.json, audit.json, parameters.yaml, controller_server.yaml, navigation_stages.json, bag/, candidate_costs.csv, snapshots/, contacts.jsonl을 보존했다.
