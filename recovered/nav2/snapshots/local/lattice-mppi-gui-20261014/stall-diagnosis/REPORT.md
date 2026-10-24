# 문 앞 정체 진단

현재 자세에서 단순 회전 또는 현재 방향을 유지한 이동으로 문에 진입하기 어렵다. 이동과 회전을 함께 조합하는 연결 경로는 통과 가능하지만, MPPI가 그 동작을 찾지 못하고 작은 속도 명령을 반복하는 상태였다. 이것이 가장 유력한 최초 정체 원인이다. 어떤 critic 하나가 이를 일으켰는지는 확정하지 못했다.

설치된 Nav2 FootprintCollisionChecker를 C++로 직접 호출해 같은 로컬 코스트맵과 실제 MPPI 경로를 검사했다.

| 검사 | lethal 판정 자세 수 |
|---|---:|
| 현재 자세 | 0 / 1 (cost 253, collision 아님) |
| 실제 경로 자세 | 0 / 29 |
| 현재 자세 연결 + 경로 보간, 1cm/1도 | 0 / 244 |
| 현재 XY에서 경로 첫 방향까지 회전 | 8 / 31 |
| 현재 yaw를 유지하고 위 위치들을 따라 이동 | 35 / 244 |

경로 첫 방향과 로봇 방향 차이는 약 19.3도. AMCL/GT 차이는 해당 스냅샷에서 XY 2.26cm, yaw 0.17도였다. 이 수치는 코스트맵상 운동 가능성 검사이며 실제 Gazebo 충돌/접촉 시험이 아니다. 실제 경로와 로컬 코스트맵 stamp 차이는 약 22ms이고 TF는 최신값이다.

정체 지속의 확정 원인: progress_checker.movement_time_allowance=1500초(시뮬레이션 시간), failure_tolerance=-1이었다. BT는 주기적 재계획 없이 기존 경로의 유효성 실패 또는 FollowPath 실패 시에만 재계획한다. MPPI가 유효한 작은 명령을 계속 내면 복구를 오래 시작하지 않는다. yaw 0.25rad 이상 변화도 진행으로 인정하므로 제자리 방향 변경이 진행 기준을 갱신할 수 있다. [설치 버전 1.1.20 PoseProgressChecker](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/plugins/pose_progress_checker.cpp).

휠 제한은 관찰한 명령에 PASS/scale=1을 반환했다. pan 및 3D guard의 정지 로그는 이 실험에서 shadow 출력이므로 실제 정체 명령의 원인이 아니다. 현 위치 새 계획 요청도 약 0.018초에 성공했으므로 문 앞 정체를 계획 연산량 부족으로 설명할 근거는 없다.

장애물 비용 감소, 회전 벌점 제거, 샘플 분산 증가, 로컬 depth layer 비활성화, 경로 방향 정렬 비활성화, temperature 감소, 경로 추종 가중치 증가를 각각 시행했다. 어느 것도 문 통과를 만들지 못했다. 상세 이동량은 ablation-tests.csv. 각 단계가 같은 초기 상태에서 반복된 시험은 아니며 동적 파라미터 변경 시 optimizer가 reset되므로 개선 성능 순위로 해석하지 않는다. temperature 감소 단계는 목표에서 멀어지는 이동이었다.

처음의 14/32 및 자체 Bresenham 보간 검사 5/244 충돌 수치는 설치된 Nav2 검사 결과와 일치하지 않아 원인 근거에서 제외했다. 정확한 native 경로 보간 결과는 0/244다.

마지막 실험에서 PathAlignCritic.offset_from_furthest=0을 주자 controller_server가 SIGSEGV(-11)로 종료됐다. 이 값은 원래 2였고 원래 정체 당시에는 프로세스가 살아 있었으므로 별도 실험 유발 오류다. source의 0개 path segment 처리에 빈 벡터/인덱스 경계 문제가 의심되지만 stack trace는 수집하지 않았다. 이 종료를 원래 주행 성능으로 비교하면 안 된다. 설정은 영구 변경하지 않았고 runtime-v3-restored/domain230에서 기존 설정 GUI와 주행을 HOME부터 복구했다. AMCL 0.01m/0.01rad 및 1sim초 nomotion update도 새 도메인으로 다시 실행했다.

다음 수정 방향:

1. 문 가까이서 막힌 뒤 회전하는 대신, 넓은 지점에서 진입 자세를 먼저 맞추거나 후퇴-정렬-진입 동작을 만들 것.
2. 정체 감지를 1500초에서 현실적인 수 초~수십 초로 복구하고 경로 진행량을 함께 감시할 것. 주기적 재계획/후퇴 복구를 추가할 것.
3. 다음 재실험은 per-critic 비용과 candidate yaw를 기록해 어느 비용/샘플링 조건이 복합 동작을 차단하는지 구분할 것.

휠5rad/s, 정확한 물리 footprint/padding0, 도착30cm/10도는 유지했다. production 코드/설정 변경 및 git push는 하지 않았다. 이전 주행의 최종 action_status=6은 마지막 실험 유발 종료이며, 전체 주행 결과에는 여러 진단 개입이 들어 있으므로 baseline 성능으로 사용하지 않는다.
