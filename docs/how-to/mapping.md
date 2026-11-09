# 지도 만들기와 저장 지도 주행

LiDAR, 휠 피드백과 TF를 확인한 뒤 매핑 세션을 시작합니다. 저장된 지도와 원점을 확인하고 주행 구성에 연결합니다.

```mermaid
flowchart LR
  Sensors["LiDAR + 휠 피드백"] --> Ready["scan · odom · TF 준비"]
  Ready --> Session["매핑 세션 / Cartographer"]
  Session --> View["지도 · 스캔 · 궤적 관측"]
  Session --> Stop["입력 차단 · STOP 확인"]
  Stop --> Save["지도 저장 · Logger flush · 종료"]
```

[공통 환경 준비](../RUNNING.md#공통-준비) · [검증 선택](../TESTING.md)

## 지도·스캔·위치를 함께 보기

![기록된 지도, LiDAR 스캔, 로봇 TF와 궤적을 겹쳐 본 예시](../assets/examples/mapping-poster.png)

실물 bag에 기록된 레이어를 같은 시각으로 표시한 예시입니다. 지도의 회색·흰색·검은색, 스캔 점과 로봇 좌표축을 구분해 봅니다. 궤적은 이동 이력이며 현재 위치는 TF로 표시합니다.

<details><summary>기록 재생 GIF 보기 · 6.7 MB</summary>
<img src="../assets/examples/mapping-replay.gif" alt="기록된 SLAM 레이어를 시간순으로 재생한 애니메이션" loading="lazy">
<p>기록을 6초 간격으로 표본화한 약 60배속 재생입니다. 새 SLAM 계산이나 완전한 원본 재생이 아니며, 기록 누락은 유지됩니다.</p>
</details>

## 실물 휠 odometry와 2D LiDAR 매핑

1. [개발환경](../DEVELOPMENT_SETUP.md#실물-매핑-의존성)의 외부 Agent와 LiDAR 드라이버를 준비하고 `make build-hardware-mapping`을 실행한다.
2. 저장소 루트에서 `make run-hardware-mapping`으로 실행할 수 있다. 기존 base, Nav2, 조종기와 중복 실행하지 않는다.
   별도 터미널에서 직접 launch하려면 빌드한 overlay를 먼저 source한다.
3. 현재 센서 장착과 일치하는 [프로필](../../ros2_ws/src/cleany_bringup/docs/hardware-mapping.md)을 선택한다.

   ```bash
   ros2 launch cleany_bringup hardware_mapping.launch.py \
     profile:="$(ros2 pkg prefix --share cleany_bringup)/config/mapping_legacy.yaml"
   ```

4. 로그의 조종 주소와 state_root/web-token 키로 접속한다. 주소의 127.0.0.1만 노트북 LAN 주소로 바꾼다.
   연결 후 지도 초기화를 누르고 running 상태를 확인한 뒤 휴대폰으로 조종한다. 초기화 접수는 완료를 뜻하지 않는다.
5. Ctrl+C로 종료하면 입력을 막고 STOP, 지도/pbstream 저장과 Logger flush를 확인한 뒤 장치를 종료한다.
   지도는 기본 `~/.local/state/cleany/hardware/maps/`, 녹화는 `recordings/`에 보존한다.

`state_root`, `host`, `phone_port`, `mock`은 launch 인자다. 모의 모드는 별도 ROS domain에서 사용하며
실물 장치 대신 main의 mock MCU를 실행한다. 모의 LiDAR 입력은 런타임 검사 도구가 제공한다.
새 SLAM은 IMU를 사용하지 않으며 raw scan 원본과 0.30m 마스킹을 유지한다.
프로필의 `scan_filter.half_angle_deg: 90`은 전방 180도, `180`은 실험용 전방위 360도다.
각도 변경은 해당 프로필로 하드웨어 런타임을 시작할 때 적용되며, 지도 초기화만으로는 바뀌지 않는다.
기존 Dev Monitor는 별도로 실행해 지도와 센서를 관측할 수 있다.

휴대폰만 독립 실행할 때는 같은 geometry의 base driver가 먼저 실행돼 있어야 한다.

```bash
ros2 launch cleany_teleop teleop.launch.py \
  base_config:="$(ros2 pkg prefix --share cleany_base_driver)/config/base_hardware.yaml"
```

문제가 발생하면 조종 상태 API의 `last_permission_error`, `event_log`와 매핑 상태 API의
`status_age_ms`, `error_code`, `error_file`을 먼저 확인한다. 기본 실행의 조종 사건은
`~/.local/state/cleany/hardware/teleop-events.jsonl`, 매핑 오류 JSON은
`~/.local/state/cleany/hardware/maps/errors/`에 남는다. `state_root`를 바꿨다면 해당 경로를
따른다. 오류 JSON을 쓰지 못한 경우 `state_root/mapping.log`에서 traceback을 확인한다.

### 오프라인 SLAM 비교용 원본 bag

Logger의 가공 기록과 별도로 연속적인 센서 입력을 확보하려면 Jetson에서 빌드한
ROS overlay를 source한 뒤 다음 도구를 실행한다. Python 관측기의 큐를 거치지 않고
네이티브 rosbag2가 `/scan_raw`, `/odom`, `/tf`, `/tf_static`, `/base/wheel_state`,
`/joint_states`를 저장한다. 조종 명령과 기존 지도 출력은 입력에 포함하지 않는다.
SQLite에는 `resilient` preset을 적용하며 기본 native cache는 64 MiB다.

```bash
python3 tools/validation/record_mapping_bag.py \
  --output "$HOME/.local/state/cleany/mapping-bags/trial-$(date +%Y%m%dT%H%M%S)" \
  --duration-s 180
```

180초가 지나거나 Ctrl+C를 누르면 해당 recorder만 SIGINT로 종료해 cache와 metadata를
마감한다. 기존 bag 경로는 덮어쓰지 않는다. 같은 비교 주행 중에는 지도 초기화로
odom 원점을 바꾸지 않는다. 사용한 하드웨어 프로필과 Cartographer 설정도 함께 보존한다.
원본 bag에서도 시각 공백과 메시지 수를 확인한 뒤 재처리에 사용한다.

## 저장 지도 Navigation

```bash
make build-navigation
source "$(make -s print-navigation-install-base)/setup.bash"
ros2 launch cleany_navigation navigation.launch.py --show-args
```

[Launch](../../ros2_ws/src/cleany_navigation/launch/navigation.launch.py)에서 지도, backend, sensor 옵션을 확인합니다. `map → odom`의 소유자는 하나만 선택하므로 Mapping과 AMCL을 함께 실행하지 않습니다.

주행 명령은 Mission의 Navigator/Nav2가 생성합니다. Runtime은 `/cmd_vel`을 직접 만들지 않으며, 안전 경로와 sensor 만료 규칙은 [Navigation reference](../../ros2_ws/src/cleany_navigation/docs/safety.md)를 따릅니다.

## 기존 지도 병행 확장

`tools/validation/map_extension/extend.py`는 실행 중인 센서의 `/scan`과
`/odom`, TF를 읽어 저장된 Cartographer 상태에 새 궤적을 추가한다.
기존 궤적은 고정하고 `/map_extension` namespace에 지도와 추정 자세를
발행한다. 기존 `/tf`를 전용 `/map_extension/tf`로 단방향 전달하며 Cartographer도
그 전용 토픽에만 연결한다. 전역 TF와 주행 명령은 발행하지 않으므로 AMCL/Nav2와 함께 실행할 수 있다.
입력은 지도 이미지가 아닌 `.pbstream`이다. 시작 시 현재 `map -> base_link`
자세와 저장 상태의 원점을 이용하므로 기존 AMCL 위치가 맞아 있어야 한다.
확장 영역에서 기존 AMCL 지도가 부족해져도 확장 Cartographer는 자체 추정을
계속한다. 자동주행 지도 교체는 별도 작업이며 수동조종으로 확장 영역을 주행한다.

ROS 환경과 현재 workspace overlay를 적용한 터미널에서 실행한다.
`--origin-reader`에는 `tools/validation/map_extension/read_origin.cpp`를
해당 Cartographer 개발 라이브러리로 빌드한 실행 파일을 지정해야 한다.
출력 폴더는 아직 존재하지 않는 경로를 사용한다.

```bash
python3 tools/validation/map_extension/extend.py \
  --source /path/to/seed.pbstream \
  --output /path/to/local-data/runs/mapping/YYYY-MM-DD_HHMMSS_map-extension \
  --origin-reader /path/to/read_origin
```

출력 폴더에는 입력 사본 `seed.pbstream`, 원점과 시작 자세 `session.json`,
60초 주기의 `latest.pbstream`, `map.yaml`, `map.pgm`을 저장한다.
종료할 때는 실행 터미널에서 `Ctrl-C`를 누른다. 궤적 종료와
`final.pbstream` 저장은 종료 로그와 출력 파일로 확인한다.
RViz에서는 `/map_extension/map`과 `/map_extension/tracked_pose`를 선택한다.
원본 지도와 기존 로거는 변경하지 않는다.

확장 RViz는 `/tf`를 `/map_extension/view_tf`로 remap해 실행한다.
`tools/validation/map_extension/view_tf.py`를 함께 실행하면 LiDAR와 odometry를
확장 SLAM의 자세 기준으로 표시한다. 궤적과 지도는 `/map_extension` 토픽을
선택한다. 제약선, raw scan, scan-matched point cloud는 표시 항목에서 켤 수 있다.

확장 중 센서/장치 재시작이나 odom 원점 초기화를 하면 기존 궤적을 계속 사용할 수 없다.
확장 입력은 `/map_extension/odom`으로 전달되며, 원본 `/odom`에 2초 이상 공백,
샘플 간 0.25m 초과 이동, 0.35rad 초과 회전 또는 역행 시각이 있으면 전달을 차단하고
현재 확장 세션을 종료 저장한다. 임계값은 `extend.py --odom-max-*` 인자로 지정한다.
문제를 해결한 후 깨끗한 원본 상태 또는 마지막 정상 저장 상태로 새 확장을 시작한다.


Phone Omni의 횡/대각 선호는 `nav2_phone_omni.yaml`의 `FollowPath.DirectionalMotionCritic`
두 가중치로 조정한다. 현재 시험값은 lateral=1, diagonal=20 cost/m이며 controller 재구성 시 적용된다.
이동 1m당 전후진 0, 횡이동 1, 45도 대각이동 20.5가 된다.
슬립 복구 BT와 자동 학습을 연결하는 설정은 없다. 로거의 motion 항목을 켜면
`/navigation/slip`의 위치/명령 문맥과 `/navigation/traction_recovery`가 기존 bag에 저장된다.
