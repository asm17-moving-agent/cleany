# DWB 완화 단계 재현

보수적 결과는 상위 directory에 보존했다. 이 단계의 source는 `source_manifest.json`의 local commit44c083a이며 `final.patch`는 conservative538e923 대비 patch다. 물리 로봇 적용이나 push는 하지 않았다. 같은 domain225/ports22551,22552의 다른 실험이 없는 상태에서 실행한다.

전용 checkout `/home/ehdrms/workspace/cleany/controller-opt-dwb`의 branch `codex/dwb-optimization-20261013`을 사용한다. ROS2 Humble, Nav2 1.1.20, Gazebo Fortress6.18 ARM64 container, feat-runtime-integration underlay, 별도 navigation/measurement build 경로는 상위 REPRODUCE.md와 같다. navigation은 다음으로 빌드 및 검증한다.

```bash
distrobox enter ros2-humble -- bash -lc 'source /home/ehdrms/workspace/cleany/cleany-data/builds/feat-runtime-integration/install/setup.bash; export ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1; cd /home/ehdrms/workspace/cleany/controller-opt-dwb; make test-navigation CLEANY_BUILD_ROOT=/home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb'
```

고정 최종5개는 각각 새로운 simulator에서 순서대로 실행한다. output에는 새 이름을 넣고 기존 결과를 덮어쓰지 않는다.

```bash
distrobox enter ros2-humble -- bash /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/relaxed61_20261013/run.sh /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/relaxed61_20261013/final_patch.yaml --output /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/relaxed61_20261013/reproduction_NEW_UNIQUE_NAME
```

대표 1개만 실행할 때 `--goal-index 1` 등을 추가한다. `run.sh`가 source provenance, domain225, loopback discovery, ports22551/22552, RTF1을 고정한다. simulator partition은 실행마다 다르다. 240sim초 초과는 기록만 하고 1200sim/1800wall초를 hard timeout으로 쓴다. progress allowance1200s, failure_tolerance=-1은 초기 control/progress 종료를 완화한다. goal checker15cm/10도는 유지한다.

`run_relaxed.py`가 prepare 결과의 외부 YAML/launch만 수정한다. source guard에는 변경이 없다. Nav2 raw 출력은 wheel cap4.99rad/s relay를 통해 Gazebo에 전달한다. pan/freshness, Collision Monitor, voxel guard는 `/shadow/cmd_vel`에 출력하며 실제 주행은 제어하지 않는다. 노드와 센서는 계속 실행된다. Gazebo 전달 guard의 axis 한계를 x/y1m/s, yaw2rad/s로 높이고 timeout5s로 설정해 유효5-wheel envelope가 .3m/s backend box에 잘리지 않게 한다. controller의 full2s collision rollout과 기존 footprint/path critics, actual robot footprint/geometry, padding.02, inflation.8은 유지한다. RotateToGoal score lookahead만 1.0s다.

31개 robot collision에 ContactSystem/sensor를 추가하고 모든 contact topic을 bridge/bag에 기록한다. update_rate20 설정에도 Fortress physics timestep.002s에 따라 persistent contact가 약500Hz로 발행됐다. JSON의 wheel-ground contact만20Hz로 줄이고 환경/body contact와 raw bag은 축소하지 않는다. GT는 contact receive 시점 최근 샘플이며 원자적 동시 pose가 아니다. depths 배열은 비어 있으므로 침투 깊이는 null이다. continuous minimum clearance도 계측하지 않아 null이다. 별도 contact_positive_control은 의도한 body overlap 검출 진단이며 비교 표에서 제외한다.

```bash
distrobox enter ros2-humble -- bash -lc 'python3 /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/relaxed61_20261013/analyze_relaxed.py /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/relaxed61_20261013/final5_v1'
```

raw bag/log, runner/support snapshots, live params, loaded libraries와 source SHA256를 각각 보존한다. CPU는 한 코어 기준 wall CPU%, 같은 호스트 관측치이다. smoothness는 GT twist20Hz sim 보간, centered5point smoothing, 두 gradient, 양쪽.25s trim이다. 중단/timeout은 숨기지 않는다. 합산 점수는 없다.
