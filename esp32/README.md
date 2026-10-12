# ESP32-S3 micro-ROS motor controller

- 역할: native USB micro-ROS·encoder·PWM/DIR 기반 4륜 제어
- Target: ESP32-S3 N32R16V, octal flash 32 MB / PSRAM 16 MB
- PlatformIO environment: esp32-s3-microros
- GPIO: [PINMAP](PINMAP.md). 버전 고정: micro_ros.lock.json

| 경로 | 역할 |
|---|---|
| src/ | firmware·motor core |
| host_tests/ | 장치 없는 C++/Python 검사 |
| scripts/ | PlatformIO·고정 dependency bootstrap |
| microros_smoke/ | micro-ROS/type-support 링크 검사 |
| pcb/ | KiCad·로컬 library |
| micro_ros/, .venv/ | 생성 dependency·빌드 도구 |

- 도구 이동 후: make firmware-setup으로 Git wrapper 링크 갱신
- Git wrapper: component root·하위 폴더 clone 모두 revision 고정

## Runtime

```text
base/wheel_command → USB XRCE-DDS / rclc callback → latest-value mailbox
                  → motor task: ENABLE / deadline / PI + feed-forward / PWM
encoders → motor task → base/wheel_state → ROS base driver / odometry
```

- 통신 task: USB·ROS entity·Agent 재연결
- Motor task: 독립 5 ms, motorMutex 안에서 명령 검사·PWM 갱신
- 부팅: PWM=0, disabled. Zero ENABLE 후 VELOCITY 허용
- 계약: [protocol v2](../ros2_ws/src/cleany_base_interfaces/docs/protocol.md)
- Sequence high-water: STOP·재연결 뒤 유지
- 거부·반복 ENABLE: deadline·이동 목표 갱신 없음
- STOP latch: 대기 명령 삭제; 새 sequence로 오래된 ENABLE 차단
- Deadline: 수신 후 최대 250 ms, 명령 처리 전 검사
- 만료·단절: 허가 해제. 재개 시 새 ENABLE 필요
- Wheel ceiling: 10 rad/s, 목표 0.1 rad/s 단위 반올림
- PWM: 20 kHz, 5 ms당 최대 1%p 변화
- 방향 반전: 측정 ≤0.5 rad/s 상태 50 ms 후 DIR 변경
- STOP·만료·단절: controller reset·PWM 0 요청
- Wheel 배열: FL/FR/RL/RR → PCB M1/M2/M4/M3, mapping {0,1,3,2}
- 극성: MCU에서만 적용. Encoder: 출력축 1회전=3172 counts
- State: 50 Hz, best-effort/volatile/keep-last-1
- MCU µs: monotonic. Boot ID: reboot baseline 식별
- USB: Agent용 framed XRCE-DDS 전용
- Agent ping: 100 ms 주기·20 ms timeout; 실패 시 STOP latch → entity teardown → 재연결

## 휠별 초기 PI·FF

- 설정: src/main.cpp의 kFl/Fr/Rl/RrControllerConfig
- 필드 순서: FF scale / Kp / Ki / PWM limit
- FF: target / FF scale × PWM limit
- 반영: 해당 휠 설정 수정 → rebuild → upload
- 측정: 2026-10-02, 네 바퀴 공중 무부하·동시 정/역방향
- 방법: PI off, PWM 0→100%, 5% 간격·1.5초

| 휠 | FF scale (rad/s) | Kp | Ki | PWM 상한 (%) |
|---|---:|---:|---:|---:|
| FL | 12.4261 | 4.655 | 9.309 | 100 |
| FR | 12.1178 | 4.711 | 9.421 | 100 |
| RL | 11.6403 | 4.864 | 9.728 | 100 |
| RR | 11.8128 | 4.783 | 9.566 | 100 |

- FF 추정: PWM≈k×속도, FF scale=100/k; 마찰 잔차는 PI 보정
- PI 추정: gain K·시정수 τ·delay L, 기존 50 ms 속도 필터 포함
- 식: Kp=clamp(0.6/K,2,8), Ki=Kp/max(0.5,4(τ+L))
- Upload 후: 네 휠 동시 정/역 0.5/2/5/10 rad/s, 각 5초
- 결과: 마지막 0.5초 평균 오차 모두 ≤4.5%; 최종 disabled·PWM 0·fault 0
- 적용 범위: 해당 전원·공중 무부하. 지면 하중 별도 검증

## 빌드·장치 없는 검사

- 명령: 저장소 루트. 호스트 호출 시 Ubuntu 22.04 ros2-humble Distrobox 사용

```bash
make firmware-setup
make test-micro-ros-setup test-motor-core
make firmware-smoke
make firmware-build
make build-base test-base
make micro-ros-agent-build
```

- Type support 원본: ros2_ws/src/cleany_base_interfaces
- Smoke/runtime: cache 공유·순차 빌드
- Interface·metadata·환경 변경: type support 재생성
- sdkconfig.defaults: octal memory·FreeRTOS 1000 Hz·USB transport
- DOUT image header: bootloader OPI 활성화 지원
- Clangd compile DB: Distrobox 안에서 실행

```bash
esp32/.venv/bin/platformio run -d esp32 -e esp32-s3-microros -t compiledb
```

## Upload·통신

- Upload: 장치 선택 + 명시적 확인

```bash
make firmware-upload CLEANY_ESP_PORT=/dev/serial/by-id/<device> CONFIRM_UPLOAD=1
```

- Agent/ROS overlay 적용, DDS domain 0

```bash
source /opt/ros/humble/setup.bash
source esp32/micro_ros/agent/install/local_setup.bash
source "$(make -s print-install-base)/local_setup.bash"
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent serial --dev "$CLEANY_ESP_PORT" -b 115200
# Another terminal with the same overlays/domain:
ros2 topic echo /base/wheel_state --qos-reliability best_effort
```

- Bare MCU 확인: nonzero boot ID, sequence/timestamp 증가, disabled, 목표/PWM 0, fault 없음

## 실물 인수 조건

- 감독자·사람 없는 구역·검토한 제한값·접근 가능한 물리 E-stop 준비
- 바퀴를 띄워 startup 0·회전 방향·encoder 부호 확인
- Enable 후 새 저속 명령: 전진/좌측/yaw·PI·PWM slew·반전 dwell 확인
- STOP·만료·driver/Agent 종료·USB 단절·MCU reboot의 정지 시간 측정
- 재연결: 새 enable + 새 명령 필요
- Odometry·TF 소유권: [Base 계약](../ros2_ws/src/cleany_base_driver/docs/command-and-odometry.md)
- 기록: revision·설정·하중/바닥·부호·주기·정지 시간

## KiCad

- 프로젝트: [motor_controller.kicad_pro](pcb/motor_controller.kicad_pro)
- Symbol/footprint: project-local table
- Espressif 출처·라이선스: [libraries](pcb/libraries/README.md)
