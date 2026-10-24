# DWB 재현

원래 비교 source는 b3ca816ba420452ee7960837bc16ec74148e07b0이다. 보수적 결과의 전용 checkout은 `/home/ehdrms/workspace/cleany/controller-opt-dwb`, local commit `538e9231ac68a4a95ebed4c7961d995ce539fc26`이다. push하지 않았다. 다른 checkout의 dirty 변경은 작업 대상이 아니다.

완화 작업 후 현재 branch HEAD는 `44c083a`이다. 보수적 source/library hash까지 재현하려면 실행 중인 실험이 없는 상태에서 이 전용 checkout만 `git switch --detach 538e9231ac68a4a95ebed4c7961d995ce539fc26`으로 고정하고 아래 build를 수행한다. 보수적 실행 완료 후 `git switch codex/dwb-optimization-20261013`과 navigation rebuild로 완화 버전을 복원한다. main checkout은 변경하지 않는다. 완화 설정과 재현은 `relaxed61_20261013/REPRODUCE.md`에 별도 기록한다.

ARM64 `ros2-humble` container, Nav2 1.1.20, Gazebo Fortress 6.18을 사용한다. 기본 simulator/base dependencies는 기존 feat-runtime-integration underlay이고, navigation plugin과 계측 wrapper는 별도 build/install이다. 원래 main과 underlay library를 편집하지 않았다.

```bash
distrobox enter ros2-humble -- bash -lc 'source /home/ehdrms/workspace/cleany/cleany-data/builds/feat-runtime-integration/install/setup.bash; export ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1; cd /home/ehdrms/workspace/cleany/controller-opt-dwb; make test-navigation CLEANY_BUILD_ROOT=/home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb'
```

계측 wrapper의 원본은 `measurement_src/cleany_benchmark`에 있다. 전용 build에는 이미 빌드돼 있다. 새 환경에서는 다음 명령을 사용한다.

```bash
distrobox enter ros2-humble -- bash -lc 'source /opt/ros/humble/setup.bash; source /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/install-navigation/setup.bash; colcon --log-base /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/measurement_ws/log build --base-paths /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/measurement_src --build-base /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/measurement_ws/build --install-base /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/measurement_ws/install --symlink-install'
```

최종 5개 목표는 각각 simulator를 새로 시작하고 GT 초기 pose [0,0,0]으로 실행했다. 목표 순서는 straight, turn_arrival, chair_gap, far_heading, distant이다. 각 240 simulation seconds / 360 wall seconds watchdog을 유지한다. 결과 directory가 있으면 새 이름을 사용해야 하며 덮어쓰지 않는다.

```bash
distrobox enter ros2-humble -- bash /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/run.sh /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/final_patch.yaml --output /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/reproduction_NEW_UNIQUE_NAME
```

대표 목표 하나만 재현할 때는 `--goal-index 0` 또는 `--goal-index 2`를 추가한다. 최초부터 반복 캠페인을 돌리지 않는다. `run.sh`가 domain 225, loopback discovery, 고유 Gazebo partition, 독립 discovery ports 22551/22552 및 RTF 목표 1을 설정한다. 동시에 같은 domain/ports를 쓰지 않는다.

```bash
distrobox enter ros2-humble -- bash -lc 'python3 /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/analyze_runs.py /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/final61_20261013_v1'
```

`run_dwb.py`, `analyze_runs.py`, `final_patch.yaml`, `final.patch`가 재현 입력이다. 개별 run에 runner snapshot, parameter patch, source/library SHA256, 실제 loaded library 경로, live parameters와 raw bag/log를 보존했다. 전용 checkout/계측 build만 재사용하며 이전 evidence를 삭제하거나 덮어쓰지 않는다.
