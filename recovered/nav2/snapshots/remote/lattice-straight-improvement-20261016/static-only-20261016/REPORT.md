# Lattice 정적 경로만 비교

MPPI·Gazebo·재계획·localization 갱신을 실행하지 않았다. 설치된 Nav2 1.1.20의 AStarAlgorithm<NodeLattice>와 GridCollisionChecker를 동일 frozen global costmap에 직접 실행했다. 실제 planner pose 결과만 그렸고 smoothing/직선 치환은 하지 않았다.

## 결론

현재 refined profile도 기록된 start pose에서 2번/4번에 불필요한 곡선이 남는다. 실제 주행 궤적이 아니라 정적 플래너 출력 자체의 현상이다. 곡선 primitive를 제외하면 최대 편차가 줄지만 5cm 격자/8방향 동작의 꺾임이 남고, 2번 길이가 증가해 완전한 해결은 아니다. 이 진단 후보를 주행 기본값에 반영하지 않았다.

## 동일 시작점에서의 비교

recorded는 원래 실험의 최초 plan start를 고정했다. 모든 profile의 start/goal/costmap/footprint가 같다. original_exact는 구형 profile에도 goal tolerance=0을 적용해 현재 profile과 동일한 goal cell/yaw에 도달하도록 통제했다. original_profile 항목에는 기존 tolerance=5cm도 별도 보존했다.

| 루트 | 기존 exact 편차 cm | 현재 편차 cm | 곡선 제외 편차 cm | 현재 길이 m | 곡선 제외 길이 m |
|---|---:|---:|---:|---:|---:|
| 1 | 18.23 | 4.74 | 4.74 | 1.021 | 1.021 |
| 2 | 15.00 | 9.10 | 4.87 | 1.989 | 2.071 |
| 4 | 22.00 | 9.60 | 4.94 | 4.439 | 4.421 |
| 6 | 13.81 | 4.60 | 4.60 | 0.671 | 0.671 |

최대 편차는 해당 plan의 시작–끝 직선에서의 수직 거리다. 1/6은 주로 5cm 위치 차이를 짧은 대각/직각 연결로 보정한다. 2/4는 현재 control set의 곡선 동작을 활용해 위치 보정과 목표 자세 변경을 섞는다. 곡선 동작은 원래 272개 중 arc_length>=0.001인 64개만 제거했고, 나머지 208개의 직선·횡이동·제자리 회전 동작과 costmap collision checking을 유지했다.

## 시작 위치 차이 분리

home은 실제 명목 home 좌표 [-1.865,-4.705,1.5708]를 모든 profile에 고정한 보조 비교다. 이때 목표와 동일 축 위에 놓이는 1/2/4/6의 현재 profile은 모두 XY 편차 0cm를 냈다. 이를 주된 결과로 제시하면 실제 입력에서 남는 굴곡을 가리므로 recorded 결과를 주 그림으로 사용한다. start의 5cm 안팎 격자 차이가 path 선택에 민감하게 작용한다는 증거다.

## inflation 비용 분리

| 루트 | 현재 비용 포함 편차 cm | cost_penalty=0 편차 cm |
|---|---:|---:|
| 1 | 4.74 | 8.24 |
| 2 | 9.10 | 9.10 |
| 4 | 9.60 | 9.60 |
| 6 | 4.60 | 4.60 |

cost_penalty=0에서도 collision checking은 유지했다. 특히 2/4의 결과가 변하지 않아 inflation 비용만 낮추는 것으로 해결되지 않는다.

## 직선 기준 경로 검사

planner가 생성한 경로와 구분한 기하학적 기준 경로를 추가 검사했다. XY를 시작–목표 직선으로 잇고 차체 yaw는 시작 자세로 유지한 뒤 목표에서 요청 yaw로 회전한다. 설치된 native GridCollisionChecker의 full polygon 검사를 사용했으며, 모든 query pose를 <=1cm translation / <=2deg rotation 간격으로 표본화했다. native checker 내부 orientation cache는 72bin(5deg)다. 네 루트 모두 표본 검사에서 충돌이 검출되지 않았다. 이 기준 경로는 planner output으로 그림에 대체 삽입하지 않았다. 표본 검사는 연속 swept volume이나 실제 주행 성공의 증명이 아니다.

즉 정적 검사에서 직선 대안이 존재하는데도 현재 Lattice 출력은 더 휜 동작을 고른다. 다음 조정 대상은 MPPI가 아니라 Lattice의 control set/거리 heuristic/경로 비용이며, 곡선 제거만을 최종 해법으로 채택하지 않는다.

## 파일

- media/current-vs-no-curves-static-xy-2rows-4cols.png: 위 현재, 아래 곡선 primitive 제거. XY 중심선만 표시.
- media/current-vs-no-curves-static-footprints-2rows-4cols.png: 동일 계획에 planner yaw의 실제 footprint를 겹침.
- media/recorded-static-xy-3rows-4cols.png: 기존 exact / 현재 / 곡선 제거 세 profile.
- media/home-static-xy-3rows-4cols.png: 이상적 home 시작점의 보조 비교.
- comparison.json / comparison.csv: 2가지 시작 조건 × 4루트 × 6profile = 48개 정적 결과.
- direct-check-summary.json: 별도 직선 기준 경로 native checker 표본검사 결과.
- 원격 동일 상대 경로에 raw binary frozen costmap, input/output JSON, 검사 코드 및 네이티브 바이너리를 보존.
