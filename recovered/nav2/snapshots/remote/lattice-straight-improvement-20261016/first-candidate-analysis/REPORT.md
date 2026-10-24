# Lattice 직선·횡이동 개선 비교

대상: 1 straight, 2 turn_arrival, 4 far_heading, 6 lateral. 원래 8-route 실험은 보존하고, 기존 설정 4회와 개선 설정 4회를 같은 원격 호스트에서 순차 재주행했다. 각 조합은 한 번의 관측이며 반복 신뢰구간은 없다.

## 원인과 변경

- 기존 16-heading / 144개 omni 프리미티브에서 90° 자세의 직진·횡이동 endpoint가 15cm 단위였다. map resolution 5cm가 곧 5cm 이동 동작을 의미하지 않는다. 초기 AMCL pose와 목표의 셀 차이는 이 동작 단위의 배수가 아니므로 곡선/회전을 섞어 정확한 goal cell을 연결한다.
- 실제 15cm 우측 횡이동의 trajectory_length/straight_length는 45cm로 설정돼 있었다. 이는 거리와 탐색 비용의 비대칭이며, 의도 여부를 단정하지 않는다. 비용만 실제 거리로 고친 ablation은 2/4/6의 휨을 제거하지 못했다.
- 설치된 Nav2 1.1.20의 Lattice distance heuristic은 omni도 Dubins/Reeds-Shepp 거리를 사용한다. 기존 analytic_expansion_max_length=0 및 smooth_path=false였으므로 analytic expansion/후처리 smoothing이 이 현상을 만든 것은 아니다. 장애물 없는 native 실험에서도 휨이 재현됐다.
- 기존 곡선·제자리 회전 144개는 보존하고, 16개 자세 각각에 5cm cardinal/diagonal translation 8개를 추가했다. 총 272개, heading 수는 여전히 16개다. 새 translation은 차체 자세를 유지하고 실제 이동거리 비용을 쓴다. 대각선 길이는 약 7.07cm다.
- planner-override.yaml: lookup_table_size=0.05, rotation_penalty=0.5, reverse_penalty=1.0, non_straight_penalty=2.0, change_penalty=0.0, retrospective_penalty=0.0. 짧은 lookup window는 차체형 거리 휴리스틱의 근거리 영향 범위를 줄이는 실험 설정이며, 휴리스틱 전체를 holonomic 구현으로 교체한 것은 아니다.

## 동일 costmap 정적 ablation

native-matched-circum/는 설치된 AStarAlgorithm<NodeLattice>, GridCollisionChecker 및 동일 footprint를 사용한다. 원래 실험의 goal 전 마지막 global costmap snapshot과 최초 plan의 start pose를 고정했다. inflation circumscribed-cost threshold는 실제 footprint 및 cost_scaling_factor=6.0에 맞췄다. 실행 시점 costmap 전체 재생은 아니며 정적 원인 분리용이다. 초기 exploratory ablation 디렉터리들은 footprint-check shortcut threshold=0을 썼으므로 런타임과 동일 비용의 replay로 해석하면 안 된다.

| 루트 | 기존 계획 휨 cm | 거리 비용만 수정 | 5cm 동작만 추가 | 최종 후보 |
|---|---:|---:|---:|---:|
| 1 | 18.23 | 15.77 | 4.74 | 4.74 |
| 2 | 15.00 | 15.00 | 5.13 | 8.15 |
| 4 | 15.45 | 15.45 | 18.98 | 9.19 |
| 6 | 13.81 | 13.81 | 4.60 | 4.60 |

## Gazebo 실제 재주행

아래 전후 값은 이번에 순차 재주행한 baseline → improved다. 원래 실험 결과는 comparison.json/CSV의 original 항목에 별도로 보존했다. 계획 휨은 최초 plan 자체의 시작–끝 직선 기준이고, 실제 주행 휨은 initial GT–요청 goal 직선 기준이다. 서로 다른 좌표기준 지표를 혼합하지 않는다.

| 루트 | 최초 계획 휨 cm | 실제 GT 휨 cm | sim 시간 s | 최종 GT 위치오차 cm | 최종 GT 각도오차 deg | Nav2 status |
|---|---:|---:|---:|---:|---:|---|
| 1 | 23.31 → 0.00 | 7.15 → 2.56 | 4.44 → 3.96 | 26.90 → 26.02 | 0.81 → 0.73 | 4 → 4 |
| 2 | 12.41 → 8.15 | 14.56 → 8.78 | 15.39 → 14.27 | 6.74 → 1.24 | 8.09 → 6.03 | 4 → 4 |
| 4 | 30.00 → 0.00 | 17.32 → 5.35 | 28.96 → 28.50 | 1.57 → 3.98 | 5.81 → 5.88 | 4 → 4 |
| 6 | 13.81 → 4.57 | 6.37 → 1.01 | 2.56 → 2.31 | 28.01 → 26.60 | 1.68 → 0.76 | 4 → 4 |

## 부드러움 및 실행 조건

| 루트 | 선형 jerk RMS m/s³ | 각 jerk RMS rad/s³ | 정지 s | 실제 RTF | 제어 명령 sim 주기 mean s |
|---|---:|---:|---:|---:|---:|
| 1 | 2.03 → 1.96 | 0.55 → 0.14 | 0.70 → 0.60 | 0.42 → 0.39 | 0.02 → 0.02 |
| 2 | 1.09 → 0.78 | 1.64 → 1.91 | 0.70 → 0.60 | 0.42 → 0.39 | 0.02 → 0.02 |
| 4 | 0.68 → 0.46 | 1.40 → 0.68 | 0.65 → 0.70 | 0.38 → 0.39 | 0.02 → 0.02 |
| 6 | 1.92 → 2.54 | 0.52 → 0.17 | 0.25 → 0.65 | 0.41 → 0.45 | 0.02 → 0.02 |

MPPI compute는 TimedController의 steady wall time이며 planner 전체 시간과 구분한다. 다음은 median / p95 / p99 ms다.

| 루트 | 기존 MPPI compute | 개선 후 MPPI compute |
|---|---|---|
| 1 | 3.50 / 4.41 / 5.62 | 3.86 / 4.88 / 6.13 |
| 2 | 3.61 / 4.43 / 6.40 | 3.78 / 4.56 / 5.46 |
| 4 | 3.82 / 4.92 / 6.27 | 3.60 / 4.57 / 5.56 |
| 6 | 3.74 / 4.27 / 7.47 | 3.64 / 4.27 / 6.42 |

## 조건 검증 및 해석 한계

- comparison.json의 checks로 MPPI parameter 전체(artifact timing path 제외), loaded MPPI binary SHA, world source, occupancy map, behavior tree 및 goal의 전후 일치를 검증했다.
- 휠 command 제한 5rad/s, 도착 허용 30cm/10°, footprint padding 0은 유지했다. actual joint overshoot는 command limit과 별도로 기록한다.
- 같은 호스트에서 순차 실행했지만 실제 RTF/제어 sim 주기는 완전히 같지 않다. AMCL·샘플링·costmap 갱신 차이도 있으므로 한 번의 주행 차이를 전부 planner 변경의 인과효과로 단정하지 않는다.
- 정체로 낮아진 jerk를 개선으로 해석하지 않는다. 상태, 실제 진행, 정지시간과 함께 본다.
- 이 후보의 실제 주행 검증 범위는 요청한 네 루트다. 나머지 의자 사이·다른 방 경로의 성능을 보장하지 않는다.
- 전역 경로는 map frame이고 GT는 원 실험과 같은 world-aligned 좌표로 표시했다. 그림 배경은 static occupancy map이며 실시간 inflation 전체는 아니다.
- Contact sensor에서 관측한 non-floor/non-self 접촉을 기록한다. 관측 0건은 독립적인 연속 충돌 판정의 증명이 아니다.

| 루트 | wheel command max rad/s | joint max rad/s | 관측 접촉 기록 수 |
|---|---:|---:|---:|
| 1 | 4.88 → 4.47 | 4.39 → 4.21 | 0.00 → 0.00 |
| 2 | 4.69 → 4.98 | 4.71 → 4.94 | 0.00 → 0.00 |
| 4 | 4.74 → 4.52 | 4.74 → 4.26 | 0.00 → 0.00 |
| 6 | 3.72 → 4.39 | 3.48 → 3.99 | 0.00 → 0.00 |

## 산출물

- media/before-after-gt-footprints.png: 위 기존, 아래 개선. 최초 plan 점선 + 실제 GT 실선 및 실제 GT 풋프린트.
- media/before-after-planned-footprints.png: 위 기존, 아래 개선. 최초 plan의 실제 yaw로 그린 풋프린트 + 실제 GT 실선.
- media/before-after-2rows-4cols-2x.gif: 수신 시점에 맞춘 실제 재계획 경로와 GT를 2배속 재생. GUI 화면 녹화가 아니다.
- comparison.json / comparison.csv: original / baseline / improved를 구분한 수치.
- planner-override.yaml / grid5cm.json / make_primitives.py: 개선 프로파일 및 재현 가능한 프리미티브 생성기.
- baseline/ 및 improved/: 8회 실제 Gazebo trial 로그·계획·GT·timing·접촉 관측. 전체 rosbag은 원격 동일 결과 디렉터리에 보존.

## 구현 근거

- [Nav2 1.1.20 NodeLattice](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/src/node_lattice.cpp): trajectory_length 기반 비용, 회전 비용, Reeds-Shepp/Dubins 거리 휴리스틱.
- [Nav2 1.1.20 SmacPlannerLattice](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_smac_planner/src/smac_planner_lattice.cpp): primitive metadata와 heuristic lookup 초기화 및 footprint checker.
