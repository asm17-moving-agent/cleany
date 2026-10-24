# Humble MPPI Omni 실험 overlay

Nav2 1.1.20을 별도 workspace에 내려받고 고정된 patch를 적용한다. 기본 배포 설정을 바꾸지 않는다. 원본 tar SHA256은 `prepare_overlay.py`에 고정되어 있으며 기존 workspace의 provenance가 다르면 새 경로가 필요하다.

보수 patch는 목표 10cm 안에서 XY 탐색을 멈추고 회전 탐색을 유지하는 hysteresis(해제 13cm), sampled rollout의 초기 실측 속도 보존, fallback 시 도착 모드 유지, Savitzky-Golay 필터 뒤 constraint 재적용을 추가한다. 실제 goal checker XY tolerance보다 큰 진입/해제 반경은 활성화하지 않는다. 기존 collision critic을 통과한 회전만 선택한다.

완화 patch는 위 변경에 Omni의 결합 wheel 제한을 추가한다. `wheel_max_rad_s`가 양수일 때 `|vx|+|vy|+((wheelbase_m+wheel_separation_m)/2)|wz| <= wheel_radius_m*wheel_max_rad_s`로 twist 전체를 균일 축소한다. sampled controls를 rollout 전에, 선택 sequence를 필터 뒤에도 제한한다. 기본값 0은 추가 projection 비활성이다. footprint와 내부 collision critic은 유지한다.

## 빌드와 최소 검증

ROS 2 Humble 환경에서 저장소 루트의 Makefile을 사용한다. workspace는 저장소 밖에 둔다.

```bash
make test-mppi-experiment MPPI_EXPERIMENT_WS=/absolute/data/builds/mppi-conservative
make test-mppi-experiment MPPI_EXPERIMENT_WS=/absolute/data/builds/mppi-relaxed \
  MPPI_EXPERIMENT_PATCH="$PWD/tools/navigation_evaluation/mppi_humble_experiment/nav2-1.1.20-relaxed.patch"
```

실험 프로세스에서만 `/opt/ros/humble/setup.bash`, 필요한 Cleany overlay, 해당 MPPI `install/local_setup.bash` 순으로 source한다. 두 MPPI overlay를 동시에 source하지 않는다. `conservative_overrides.yaml`과 `relaxed_overrides.yaml`은 FollowPath override이며 robot overlay가 다시 덮어쓰지 않도록 마지막 설정에도 적용하고 실제 parameter dump를 검증해야 한다.

보수 optimizer 15개 및 smoke 3개, 완화 optimizer 16개 및 smoke 3개가 통과했다. upstream critic 23개 중 PathAlign float 합산 assertion 1개는 실패했다. 같은 바이너리를 수정하지 않은 시스템 라이브러리에 연결해 동일 실패를 재현했으며 assertion을 변경하지 않았다. 이 Makefile의 최소 검증은 optimizer/smoke에 한정된다.

## 실행 조건과 증거

2026-10-13 실험 source는 `b3ca816ba420452ee7960837bc16ec74148e07b0`, 도착 허용은 15cm/10deg, controller는 20Hz/model_dt=.05였다. 보수 설정은 원래 속도 박스와 외부 안전 경로를 유지한다. 완화 설정은 **Gazebo 전용**이며 외부 pan/freshness/Collision Monitor/voxel guard 최종 출력을 shadow로 보내고 wheel projector를 실제 `/cmd_vel`에 연결했다. .5sim초 command-loss watchdog을 유지했고 contact sensor를 추가했다. 완화 설정을 물리 로봇에 적용하지 않는다.

실험 러너, contact/wheel 계측, 실제 YAML, raw bag/log, JSON/CSV, 재현 명령은 다음 외부 증거 경로에 보존한다.

`/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/sol_resume_20261013/`

`REPORT.md`에서 Nav2 상태, ground truth 오차, 접촉 및 계측 한계를 함께 확인한다. Nav2 status 4만으로 GT 도착 허용이나 실제 비접촉을 판정하지 않는다. 단일 실행의 수치는 통계적 성공률이나 다른 CPU와의 성능 순위가 아니다.
