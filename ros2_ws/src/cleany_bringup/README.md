# cleany_bringup

Gazebo, 기존 AMCL/Nav2, Mission Runtime, Backend Gateway 및 pose relay를 함께 실행합니다.
FSM 패키지 자체에는 Gazebo 의존성을 추가하지 않습니다. 주행 node를 복사하지 않고
`cleany_navigation/amcl_nav2.launch.py`를 include하며 구 checkout에서는 simulator의
동일 launch로 fallback합니다. 저장된 지도 localization을 사용하고 SLAM은 시작하지 않습니다.

## 실행

팀 표준은 Humble/Fortress입니다. Jazzy/Harmonic은 별도 호환 환경입니다.

```bash
make build-mission-sim
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=83 ros2 launch cleany_bringup mission_sim.launch.py \
  profile:=fortress headless:=true post_mission:=wait_for_next \
  gateway_url:=ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws \
  journal_path:=/tmp/cleany-lab-missions.db \
  bridge_journal_path:=/tmp/cleany-lab-bridge.db
```

Jazzy에서는 `make ROS_SETUP=/opt/ros/jazzy/setup.bash build-mission-sim`과
`profile:=harmonic`을 사용하고 별도의 build/install/log 경로를 지정합니다.
공통 명령은 [workspace README](../../README.md)를 봅니다.

| Launch argument | 기본값 및 의미 |
|---|---|
| `profile` | ROS distro에 따라 `fortress` 또는 `harmonic` |
| `headless` | `true` |
| `map` | simulator의 `study_cafe_26cm.yaml` |
| `targets_file` | mission manager의 `study_cafe_targets.yaml` |
| `runtime_config` | mission manager의 `runtime.yaml`; 시작 시 정책 읽기 |
| `post_mission` | `return_home` 또는 `wait_for_next`; 기본 `return_home` |
| `gateway_url`, `journal_path`, `bridge_journal_path` | mission runtime launch에 전달 |
| `enable_bridge`, `enable_telemetry` | 기본 `true` |
| `pose_url` | 빈 값이면 Gateway URL에서 같은 host의 `/pose/ws`를 파생 |
| `sim_partition` | `cleany-mission`; Gazebo transport 격리 |

지도 filename stem과 targets의 `map_id`가 다르면 시작을 거절합니다. home과 좌석은
시뮬레이션 접근 후보이며 실환경 접근점·충전 dock·팔 도달성 검증 결과가 아닙니다.
현재 실제 home 복귀 시험은 실패했으므로 완료 결과를 보장하는 설정으로 해석하지 않습니다.
`wait_for_next`에서는 현재 위치에 대기하고 Backend가 다음 mission을 배차합니다.

기존 Nav2/Gazebo가 실행 중이면 이 launch를 중복 실행하지 말고
[mission runtime 단독 실행](../cleany_mission_manager/README.md)을 사용합니다.
pose relay도 기존 producer가 있으면 `enable_telemetry:=false`를 지정합니다.
지도 표시용 `/ground_truth/odom` relay는 simulator world 위치이며 wheel odometry나
AMCL 정확도의 증거가 아닙니다. 청소 단계는 현재 Mock입니다.
