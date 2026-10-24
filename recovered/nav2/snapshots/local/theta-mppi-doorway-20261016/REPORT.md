# Theta* + MPPI 전환 및 다른 방 경로 실패 개선

## 결과

새 주행 세 번은 동일한 18F world, 목적지 좌석 제거, GT/AMCL 초기화 방식, 실제 footprint(padding 0), wheel command 5rad/s, 도착 허용 30cm/10° 및 bounded recovery를 사용했다. 수동 경유점·GT pose 주입·장애물 삭제·충돌 검사 완화는 추가하지 않았다.

| 설정 | Nav2 | 최종 GT 거리 m | GT 방향 오차 deg | sim 시간 s | 실제 RTF |
|---|---|---:|---:|---:|---:|
| baseline | 6 | 13.7178 | 88.49 | 26.078 | 0.244 |
| xy_only | 6 | 13.6650 | 81.81 | 32.022 | 0.272 |
| clearance | 4 | 0.0335 | 9.33 | 102.346 | 0.404 |

## 원인 분리

- baseline은 이전과 거의 같은 초반 코너에서 MPPI collision-free emitted rollout을 찾지 못하고 recovery 후 status 6으로 끝났다. 최종 목표까지 약 13.7m 남아 있었으므로 실제 실패 지점은 목적지 방의 문 자체보다 앞이다.
- Theta*는 XY costmap 탐색이고, 중간 pose yaw=0을 직사각형 차체 자세로 보장하지 않는다. PathAlignCritic.use_path_orientations=false만 적용한 xy_only도 실패했다. 따라서 방향 값 추종만으로 실패 전체를 설명할 수 없다.
- 동일한 출발 전 frozen global costmap에서 실제 native Theta*와 native footprint checker를 사용한 정적 시험은, w_traversal_cost=2 경로의 yaw=0 footprint에서 lethal 표본 6개를 검출했다. 5로 변경한 경로에서는 0개였다. 경로 길이는 약 19.825→20.281m로 늘었다. 표본 검사는 전신/연속 충돌 보장이 아니고, 90° 고정 yaw에서는 다른 지점의 간섭도 남는다.
- clearance는 경로 yaw 추종을 끈 상태에서 Theta* w_traversal_cost=5를 추가한 조합이다. 기존 실패 코너를 지나 최종 목표까지 실제 주행을 완료했다. 모든 조합은 같은 최적화 MPPI binary를 사용했고, xy_only 대비 clearance는 global planner의 비용 가중치만 바뀌었다.

## 프로젝트 기본값 반영

- 작업 트리: /home/ehdrms/workspace/cleany/standard-nav2-planner (기존 dirty 파일은 source-backup에 보존).
- cleany_navigation/config/nav2_amcl.yaml, nav2_omni_motion.yaml 및 Gazebo의 legacy nav2_amcl.yaml: Theta* 기본 planner.
- Omni motion overlay: 기존 DWB 재덮기를 제거하고 MPPI Omni를 명시. PathAlignCritic.use_path_orientations=false, offset_from_furthest=2, CostCritic.consider_footprint=true.
- runtime_config.py: Humble Theta* plugin ::→/ 변환. package.xml: nav2_theta_star_planner runtime dependency.
- 공유 기본 프로필의 기존 속도/도착 checker 및 안전 체인은 유지. 실험의 완화 값은 공유 실물 기본값에 복사하지 않았다. 명시적 Axis 모드는 기존 Axis 정책을 유지한다.
- 단위 테스트 26개 통과, make build-navigation로 cleany_description/cleany_navigation 빌드 통과. local Humble에 Theta*/MPPI package 설치 확인.
- 소스 기본 프로필은 stock MPPI이고 이 Gazebo 검증은 보존된 최적화 candidate_diverse MPPI 빌드다. 이 둘을 같은 E2E 검증으로 주장하지 않는다. 실물 주행은 수행하지 않았다.

## 관측과 한계

- 각 변경 조합은 1회이며 반복 성공률을 보장하지 않는다. 나머지 7루트 전체를 이번 설정으로 재검증하지 않았다.
- 실패한 짧은 주행과 성공한 긴 주행의 sim 시간/jerk를 속도·부드러움 순위로 직접 비교하지 않는다. 실제 RTF와 명령 sim 주기는 comparison.json에 별도 기록한다.
- 모든 관측 접촉과 joint 최대값은 comparison.json에 기록한다. 명령 제한과 실제 joint overshoot는 다르며, 접촉 관측 0건이 연속 전신 무충돌의 독립적인 증명은 아니다.
- 전체 bag/contacts/candidate trace는 원격 동일 결과 경로에 보존. 로컬은 보고서·계획·GT·native 결과·주요 로그를 동기화한다.

## 산출물

- media/theta-mppi-doorway-before-after-2x.gif: 실제 GT 및 /plan을 2배속 재생; GUI 녹화가 아니다. 세 열은 baseline / yaw-only / clearance, 두 행은 전체 경로 / 초기 실패 코너 확대다.
- media/theta-static-clearance-before-after.png: 동일 frozen costmap의 실제 native Theta* 경로 전후.
- comparison.json / comparison.csv: 상태, GT, 시간, 연산량, jerk, wheel/joint, 접촉, 조건 일치 및 parameter diff.
- weight-sweep/summary.json, old-native-inspection.json, clearance-native-inspection.json: 정적·저장 snapshot의 native 기하 검사 근거.
