# 휠 속도 추종 진단

전후이동과 횡이동에서 네 휠의 목표 속도, 명령 속도, 실제 속도와 PWM을
수집하고 정상상태 추종 편차와 지면 하중 FF 추정치를 계산합니다.
순서는 FL, FR, RL, RR입니다.

## 실행

레포 루트에서 실행합니다. 실물 실행은
[베이스 드라이버](../../docs/implementation/how-to/base.md)의 Agent와
hardware 프로필 기동 절차를 따릅니다. 사람 없는 평탄한 시험 공간과
물리 emergency stop을 준비합니다.

```bash
# 합성 표본 분석만 실행: ROS 연결 및 주행 없음
bash tools/wheel_calibration/run_wheel_diag.sh --dry-run

# 기본값: 전후와 횡방향 각각 0.5 m 왕복, 0.3 m/s
bash tools/wheel_calibration/run_wheel_diag.sh \
  --pattern both --distance 0.5 --speed 0.3
```

Ubuntu 22.04가 아닌 환경에서는 `ros2-humble` Distrobox로 진입합니다.
실물 ROS domain은 0입니다. `--pattern surge|sway|both`,
`--no-round-trip`, `--odom-timeout`, `--baseline`, `--output-dir`를 지원합니다.
기본 odometry timeout은 0.15초입니다.

## 분석과 보정 기준값

- `controller_baseline.json`은 `esp32/src/main.cpp`의 휠별 FF·Kp·Ki를 보존합니다.
  테스트가 두 파일의 값 일치를 검사합니다.
- 목표가 안정되고 명령이 목표를 따라가는 표본만 정상상태 분석에 사용합니다.
  정방향과 역방향을 분리하며, 표본이 부족하면 보정 권고를 만들지 않습니다.
- 출력은 진단 근거와 검토용 설정입니다. 펌웨어 설정을 자동 변경하거나
  업로드하지 않습니다.
- 보정값을 변경하면 `esp32/src/main.cpp`와 baseline을 함께 갱신하고
  부하·노면·방향·속도별로 재검증합니다.

주행은 [오도메트리 평가 도구](../odometry_evaluation/README.md)의 공통
제어·freshness guard를 사용합니다. 중복 주행 도구와 다른 `/cmd_vel`
publisher를 검사하며, odometry 이상 시 STOP과 disable을 요청합니다.

## 결과와 테스트

기본 출력은 Git 제외 경로인 `artifacts/wheel_calibration/`입니다.
`wheel_diag_<pattern>_<timestamp>.json`에 원시 표본, 방향별 통계,
제어 설정과 펌웨어 소스 해시를 기록합니다. 통계와 matplotlib이 있으면
같은 이름의 `.png`를 생성합니다.

ROS Humble, NumPy, SciPy, matplotlib과 pytest가 필요합니다.
레포 루트에서 관련 테스트를 실행합니다.

```bash
distrobox enter ros2-humble --no-tty --clean-path -- bash -c \
  'source /opt/ros/humble/setup.bash &&
   source ros2_ws/install/setup.bash &&
   PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
   python3 -m pytest -q tools/wheel_calibration tools/odometry_evaluation'

make test-motor-core test-base
```
