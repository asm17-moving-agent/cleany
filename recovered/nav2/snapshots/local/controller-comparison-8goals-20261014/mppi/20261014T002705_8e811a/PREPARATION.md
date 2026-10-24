# MPPI 18F 8-goal preparation

READY 생성 전에는 주행 goal을 보내지 않는다. `offline_preparation/`은 proposed scene/config 생성 검사이며 `render_startup_preflight/`는 시작 센서/Nav2/actual30cm 파라미터를 확인한 preparation-only 실행이다. navigation_goal_sent=false다. campaign과 별도로 보존한다.

기존 완화 MPPI binary SHA 및 YAML을 재사용한다. 새 시작점을 GT와 AMCL에 world-aligned로 적용하고 도착 XY tolerance만 .30으로 바꾼다. 각 goal의 최종 좌표, canonical world, matching map은 공통 READY/locked JSON을 snapshot한 후 사용한다. 공유 common은 수정하지 않는다. 공통 static scene에서 복구한 의자를 포함해 목표 의자만 제거한 scene/model hash를 확인한다. URI는 소유 snapshot fixture에 materialize하며 geometry는 유지한다.

실제 command chain은 /nav2/cmd_vel → uniform coupled wheel projector → /cmd_vel이다. pan/depth freshness, CollisionMonitor, serial voxel/freshness guard 최종 출력은 /shadow/cmd_vel이다. 내부 footprint/CostCritic은 유지하며 별도 .5sim초 command loss watchdog도 유지한다. r=.0635m, wheelbase=.35m, separation=.6038m, maxwheel5rad/s; 순수 직진 한계 .3175m/s. 원래 .15/.10/.14 축 박스는 복원하지 않는다.

ROS_DOMAIN_ID=226, unique IGN/GZ partition, loopback discovery, headless server; GUI 실행하지 않는다. controller20Hz/model_dt=.05, targetRTF1, hard1200sim/1800wall. source/설정/원래 증거를 보존하며 소유 실험만 정리한다.
