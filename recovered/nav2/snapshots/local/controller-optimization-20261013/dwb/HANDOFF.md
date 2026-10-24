# DWB 인계 — 2026-10-13

사용자 요청으로 기존 최적화 작업을 취소하고 중단했다. 최종 5개 목표 검증은 수행하지 않았다. 커밋/push/reset/삭제 없음. 기존 main dirty 변경과 기준 evidence 보존.

## 소유 경로

- checkout: `/home/ehdrms/workspace/cleany/controller-opt-dwb`
- branch: `codex/dwb-optimization-20261013`, 시작 commit `b3ca816ba420452ee7960837bc16ec74148e07b0`
- build: `/home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb`
- output: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb`
- baseline: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-20261013`
- container: `ros2-humble`, ARM64. domain231, ROS_LOCALHOST_ONLY=1, IGN_IP/GZ_IP=127.0.0.1, run별 unique partition.

## 완료와 중단

- 기준 navigation 소스와 계측 wrapper를 전용 build에 빌드 완료. 기존 overlay는 공통 Gazebo/base dependency underlay로 사용.
- `run_dwb.py`, `run.sh`, `summarize.py` 준비. `sampling.yaml`은 샘플만 7×7×11에서 5×5×7로 변경. 15cm/10도, common planner/map/world/safety 유지.
- `pilot_sampling/2_chair_gap` 단일 대표 실험을 시작한 뒤 중단. 원시 bag/log 및 중단 결과 보존. 최적화 성공 증거로 해석하면 안 됨.
- checkout의 `holonomic_generator.cpp`에 measured velocity overshoot 시 command box를 rollout 내부에서 clamp하는 초안 추가, `test_axis_generator.cpp`에 경계 테스트 초안 추가. **이 수정은 아직 빌드/테스트/실행하지 않았음**. 현재 install library는 수정 전 소스이다. `preserve_translation_direction=false` 조기 return은 이 초안 clamp를 우회하므로 재개 시 검토 필요.
- README 반영과 최종 YAML/REPORT/CSV 검증은 미완료. 새 실험 시작 전에 인계 소스 변경을 검토해야 함.

## 발견사항

`baseline_pose_analysis.json`은 기존 official samples/result에서 계산했다. chair_gap 마지막 map 추정 XY오차 0.140930m, GT오차 0.172252m, GT/map 위치 차이 0.037560m. map의 15cm 경계 도착과 localization 오차가 합쳐진 사례이다. 허용오차를 조정하지 않았다. baseline 5개 도착의 GT/추정 차이는 약 3.8–7.3cm.

기준 command CSV에 /nav2/cmd_vel vx가 약 0.179874, vy가 -0.107272까지 나온다. 설정의 max_vel_x/.y만으로 명령 자체의 common box를 보장하지 못한다. HolonomicGenerator가 measured current에서 acceleration-limit한 첫 명령을 출력하므로 overshoot 전파 가능성을 발견했다. 원인 가설 및 미검증 clamp 초안으로 남긴다. baseline에는 joint 최대/단계별 box 별도 검증 필요.

기준 7×7×11은 timing trace에서 약 893–896 rollout/cycle. sample 감소 효과는 중단되어 아직 확정하지 않았다. 15cm goal checker로 GT 15cm 보장을 만들 수 없는 localization 경계는 controller 튜닝과 구분해야 한다.

## 재개 명령

빌드 및 경계 테스트 후 새 output 디렉토리 이름을 사용한다. 기존 pilot evidence는 덮어쓰지 않는다.

```bash
distrobox enter ros2-humble -- bash -lc 'source /home/ehdrms/workspace/cleany/cleany-data/builds/feat-runtime-integration/install/setup.bash; cd /home/ehdrms/workspace/cleany/controller-opt-dwb; make build-navigation CLEANY_BUILD_ROOT=/home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb'
distrobox enter ros2-humble -- bash /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/run.sh /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/sampling.yaml --output /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/pilot_sampling_resume --goal-index 2
```

## 종료 확인

종료 확인 완료: 소유 PID와 unique partition에 남은 실행 프로세스 0개. `shutdown_verification.json` 참조. 중단 result는 `KeyboardInterrupt`와 rclpy context 종료에 따른 cleanup timer 오류를 기록한다. launch/bag는 종료됐고 bag metadata와 8.6MB DB가 보존됐다. node.save 전 cleanup 오류로 samples.csv가 없을 수 있으므로 raw bag에서 복원해야 한다. 종료 전 소유 PID/명령 목록은 `shutdown_owned_snapshot.json`에 저장. 다른 채팅의 실험과 container 자체는 종료하지 않았다.
