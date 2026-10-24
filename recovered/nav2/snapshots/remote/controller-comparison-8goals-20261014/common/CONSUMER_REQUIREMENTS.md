# 공통8goals 고정 입력

READY 생성 후 goals.locked.json의 동일8 목표를 각각 fresh world에서1회 실행한다. 좌표는 world aligned이며 시작[-1.865,-4.705,1.5708]이다. 목표 좌표 변경 없음. 지정 seat의 의자1개만 제거하고 나머지는 보존했다. 최초 template에서 없던13,43은 bundled world.study에서 복원했다.

필수: map2.5cm, physical footprint/padding.02 동일, common StraightFirstPlanner 설정은 planner_params.yaml이다. max_planning_time80s, body_heading_grid_bins32, holonomic_direct_path=true, require_exact_goal=true, lattice_collision_padding.05. lattice_file은 common/omni_lattice_2p5cm.json; metadata grid_resolution만2.5cm이며 metric primitive poses/physical radius.5는 동일하다. global costmap resolution.025, local costmap 기존resolution 유지. 더 작은 footprint/추가 safety 축소 금지.

fixture/config/nav2_amcl.yaml도 공통 설정으로 갱신했다. URI는 각 호스트 실제 common 경로로 치환해야 한다. materialize_common.py의 resolve_text/materialize_goal 또는 CLI를 사용한다. 실제 Nav2 파일의 common://, fixture://, repo:// 잔존을 검사한다. 세계 canonical 파일은 그대로 실행하면 안 된다. materializer가 map/world/nav2_common/planner_params를 output에 쓴다. 각 controller의 FollowPath 설정만 자신의 기존 optimized relaxed profile을 병합한다. goal_checker와 controller plugin 내부 xy_goal_tolerance는30cm, yaw10도. GT errors는 진단값이고 Nav2 status와 별도로 보고한다.

5cm map에서는 M1 원래goal의 경로가2/20/80초 탐색에서 실패했다.2.5cm map에서는 원래좌표·yaw로23.12m full-footprint route가 검증됐다. 별도 좌표/heading 진단은 최종goal이 아니다. 협소 seat43은1.28m 거의 고정heading route로 검증되므로 multi-turn이라고 부르지 않는다. SceneInfo와 server log에서330 모델, mesh missing없음, unresolved URI없음을 확인했다. Runtime depth/memory와 움직임의 무접촉을 이 사전 static 검증으로 주장하지 않는다.

정지는 |v_xy|<.01 AND |wz|<.02rad/s, metrics는 common20Hz sim interpolation/centered5point/two gradients/양끝.25s trim.1200sim/1800wall hard timeout, RTF target1, headless only, wheel command/joint limit5rad/s. 외부 safety bypass는 기존 Gazebo relaxed 범위를 유지하고 actual params/publishers를 기록한다. host가다르면 compute/CPU 절대순위 금지. 실패/timeout/null 이유를 그대로 보고하고 score를 만들지 않는다.
