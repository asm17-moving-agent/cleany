## 공통 사전 검증 변경

이번 공통 입력은 최초5cm raster에서 다른 방 M1 경로를 찾지 못한 뒤 같은 물리 scene을2.5cm raster로 만들고, grid_resolution metadata만2.5cm인 대응 lattice(primitive metric poses/physical radius.5 불변)를 사용했다. 모든8개 지도/global costmap에 동일 적용하며 local costmap 기존resolution은 유지한다. shared planner는 탐색 예산80초/heading32개, holonomic_direct_path/require_exact_goal=true, lattice_collision_padding=.05이다. 물리 footprint/padding.02/internal collision을 축소하지 않는다. canonical physics step .001s를 유지한다. goals.locked.json과 common snapshot에서 원인/변경/검증을 확인한다. 별도 heading 진단을 최종 goal로 혼합하지 않는다. seat43은 preflight1.28m 거의 고정heading 경로이므로 multi-turn이라고 부르지 않는다.

## MPPI 설정과 안전 우회 범위

2026-10-13 최종 개선/완화 MPPI 1.1.20 빌드 및 YAML을 재사용하며 이번 controller 맞춤 튜닝은 없다. goal rotation domain10cm/해제13cm, sampling std .035/.025/.07, gamma .0003, PathAngle 비활성, PathAlign orientation 사용/offset2, time_steps80를 유지했다. 실제 controller dump 모든 FollowPath 값과 .30m/10deg 및 frequency20을 확인한다. 원래 .15/.10/.14 축별 보수 박스를 복원하지 않았다.

결합 영역은 `|vx|+|vy|+.4769|wz| <= .0635*5`이다. twist 전체 축을 동일 비율로 축소한다. sampled control을 rollout/collision 평가 전에, selected sequence를 update 및 Savitzky-Golay 뒤에 제한한다. 최종 wheel projector도 같은 IK를 적용한다. 순수 직진 상한 .3175m/s다. wheel command와 joint max는 별도 보고한다. controller 내부 footprint/feasibility/CostCritic을 유지하고 geometry 축소는 없다. Humble MPPI는 ax_max/ay_max/az_max를 읽지 않으므로 native acceleration 제한을 주장하지 않는다.

실제 chain: `/nav2/cmd_vel → coupled wheel projector → /cmd_vel`. 외부 pan alignment/머리 depth freshness/CollisionMonitor stop-slow/serial voxel-freshness 최종 출력은 `/shadow/cmd_vel`이다. 계속 실행되는 이 가드의 reason/STOP/PASS는 shadow 관측이며 실제 개입으로 해석하지 않는다. .5sim초 controller command-loss watchdog은 유지한다. progress allowance1500sim 및 failure_tolerance=-1은 hard1200sim/1800wall watchdog이 한정한다. depth/memory costmap과 internal collision은 유지한다. Gazebo 외부 우회 프로파일은 물리 로봇용 설정이 아니다.

## 계측과 불명값

GT pose는 world-aligned 좌표 그대로이며 옛 spawn-relative 변환을 제거했다. GT twist body velocity를20Hz sim time으로 보간하고 centered5-point moving average, 양끝 .25s 제거, 두 번 미분하여 jerk RMS/p95를 계산했다. acceleration RMS/p95는 JSON/CSV에 있다. 단위는 선형 m/s²/m/s³ 및 각 rad/s²/rad/s³이다. body velocity 변화 지표이며 물리 승차감/가속도 측정이 아니다. 공통 정지는 `|v_xy|<.01 AND |wz|<.02rad/s`이다. 재출발 및 yaw/vy 반전(deadband .02/.01)은 기술 지표다. 정체/실패로 jerk가 낮아져도 개선으로 판정하지 않는다.

Contact system과 모든 collision을 가진6개 robot link sensor/ROS bridge를 추가했다. 물리 collision/pose/joint geometry 불변을 확인했다. 모든 contact를 contacts.jsonl과 bag에 남긴다. JSONL streaming은 긴1200초 실행의 무한 RAM 누적을 피하는 계측 변경이며 샘플을 버리지 않는다. ground/self는 환경 접촉과 구분하고 이름 미확인 또는 ground heartbeat 없으면 null+이유다. packet 수는 접촉 횟수 자체가 아니므로 unique sim stamp에서50ms 이상 끊긴 구간을 episode로 계산한다. depth 배열이 제공되지 않으면 penetration depth는 null이다. minimum true footprint clearance는 별도 계측하지 않아 null이다. 비접촉 관측은 물리 안전 증명이 아니다.

computeVelocityCommands는 steady wall 시간이며 timing-file I/O는 측정 구간 밖이다. 활성 navigation 구간 median/p95/p99/exception/50ms 초과를 집계한다. CPU는 controller_server Linux proc delta로 one-core %, RSS는p95 MiB이다. command period와 RTF는sim/wall을 구분한다. ARM64 a16-fedora와 다른 x86 호스트의 absolute compute/CPU 순위를 계산하지 않는다. 병렬 다른 세션 부하, rendering 및 contact streaming 부하는 해석 한계다.

controller_frequency20/model_dt.05는 설정값이다. 실제 호출 및 command 간격은 sim/wall 두 시계로 보고하며, 낮은 RTF에서 설정20Hz를20Hz sim cadence로 해석하지 않는다. timeout/정체 결과의 오차는 성공 도착 정확도 점수가 아니다.

## 검증과 원시 증거

재사용한 완화 optimizer16개/smoke3개는 이전 검증에서 통과했다. upstream critic float assertion1개는 원본 시스템 라이브러리에서도 재현된 기존 한계이며 전체 suite 통과라고 쓰지 않는다. 이번에는 source/controller binary를 바꾸지 않았으므로 동일 build를 재사용하고 parameter/scene/lifecycle/loaded-library SHA를 실제 runtime에서 검증했다. prepare-only 검증과 캠페인 goal은 분리한다. run마다 초기 truth와 actual30cm/10deg,20Hz, modeldt 및 wheel cap을 확인한 뒤 goal을 보낸다. 다른 목표를 추가하거나 반복하지 않는다.

- `REPORT.md`, `results.json/csv`: 최종8개 목표 관측. 실패/timeout 포함.
- `final_validation/*`: actual config/dumps, result.json, raw bag/launch.log, GT velocity/commands/joints/resources/compute, contacts.jsonl, reason timeline, plans, scene metadata.
- `final_overrides.yaml`, `execute.sh`, `run_facility.py`, `REPRODUCE.md`: 설정/실제 runner/재현 명령.
- `common_snapshot`, `common_snapshot_manifest.json`, `provenance.json`: locked goals/canonical worlds/matching maps/환경 및 immutable hashes.
- `render_startup_preflight`: READY 전 navigation goal을 보내지 않은 actual parameter/센서 준비 검증.
- `remaining_owned_processes.json`: 종료 후 소유 domain/partition 정리 확인. 다른 세션 프로세스는 보존한다.

기존 dirty main과 이전 실험 증거를 보존하고 push하지 않았다. 다른 채팅으로 메시지를 보내지 않았다.
