# 휠 속도 추종 진단

전후이동과 횡이동에서 네 휠의 목표 속도, 명령 속도, 실제 속도와 PWM을
수집하고 정상상태 추종 편차와 지면 하중 FF 추정치를 계산한다.
순서는 FL, FR, RL, RR이다.

## 실행

레포 루트에서 실행한다. 실물 실행은
[베이스 드라이버](../../ros2_ws/src/cleany_base_driver/README.md)의 Agent와
hardware profile 기동 절차를 따른다. 사람 없는 평탄한 시험 공간과
물리 emergency stop을 준비한다.

```bash
# 합성 표본 분석만 실행: ROS 연결 및 주행 없음
bash tools/wheel_calibration/run_wheel_diag.sh --dry-run

# 기본값: 전후와 횡방향 각각 0.5 m 왕복, 0.3 m/s
bash tools/wheel_calibration/run_wheel_diag.sh \
  --pattern both --distance 0.5 --speed 0.3
```

Ubuntu 22.04가 아닌 환경에서는 `ros2-humble` Distrobox로 진입한다.
실물 ROS domain은 0이다. `--pattern surge|sway|both`,
`--no-round-trip`, `--odom-timeout`, `--baseline`, `--output-dir`를 지원한다.
기본 odometry timeout은 0.15초다.

## 분석과 보정 기준값

- `controller_baseline.json`은 `esp32/src/main.cpp`의 휠별 FF·Kp·Ki를 보존한다.
  테스트가 두 파일의 값 일치를 검사한다.
- 목표가 안정되고 명령이 목표를 따라가는 표본만 정상상태 분석에 사용한다.
  정방향과 역방향을 분리하며, 표본이 부족하면 보정 권고를 만들지 않는다.
- 출력은 진단 근거와 검토용 설정이다. 펌웨어 설정을 자동 변경하거나
  업로드하지 않는다.
- 보정값을 변경하면 `esp32/src/main.cpp`와 baseline을 함께 갱신하고
  부하·노면·방향·속도별로 재검증한다.

주행은 [오도메트리 평가 도구](../odometry_evaluation/README.md)의 공통
제어·freshness guard를 사용한다. 중복 주행 도구와 다른 `/cmd_vel`
publisher를 검사하며, odometry 이상 시 STOP과 disable을 요청한다.

## 결과와 테스트

기본 출력은 Git 제외 경로인 `artifacts/wheel_calibration/`이다.
`wheel_diag_<pattern>_<timestamp>.json`에 원시 표본, 방향별 통계,
제어 설정과 펌웨어 소스 해시를 기록한다. 통계와 matplotlib이 있으면
같은 이름의 `.png`를 생성한다.

ROS Humble, NumPy, SciPy, matplotlib과 pytest가 필요하다.
레포 루트에서 관련 테스트를 실행한다.

```bash
distrobox enter ros2-humble --no-tty --clean-path -- bash -c \
  'source /opt/ros/humble/setup.bash &&
   source ros2_ws/install/setup.bash &&
   PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
   python3 -m pytest -q tools/wheel_calibration tools/odometry_evaluation'

make test-motor-core test-base
```
