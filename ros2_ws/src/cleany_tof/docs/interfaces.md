# ToF와 정밀 접근

## 명령과 책임

`tof_enabled:=true`일 때 공통 navigation launch가 다음 경로를 구성한다.

```text
Nav2 → nav2/cmd_vel → desk_approach의 명령 선택 → motion/cmd_vel
  → 기존 pan gate → Collision Monitor → 기존 voxel/reactive guard
  → proximity/input → proximity_guard → cmd_vel → base driver → MCU watchdog
```

비활성인 기존 안전 단계를 새로 활성화하지 않는다. 기존 LiDAR, RGB-D, costmap과 3D
안전 설정을 유지한다. 정밀 접근도 위 경로를 모두 통과하며, 장애물 때문에 진행할 수
없으면 `NO_PROGRESS` 또는 timeout으로 종료한다. 기존 정지 polygon을 줄이지 않는다.

`tof_driver`는 USB와 신선도, `proximity_guard`는 순간 속도 제한, `desk_approach`는
명령 선택과 접근 Action을 소유한다. System Supervisor는 가용성, Fault, lease 회수와
Mission 정지 확인을 소유한다. 일반 장애물 감속은 System Fault가 아니다.
`tof_required:=true`인 Supervisor는 `tof_proximity` 가용성을 `BASE_MOTION`의 필수
근거로 사용한다. 센서가 복구되어도 취소된 Mission을 재개하지 않는다.

## ROS endpoint

| 노드 | 방향과 기본 이름 | 타입 | QoS와 의미 |
|---|---|---|---|
| `tof_driver` | 발행 `tof/frame` | `cleany_interfaces/msg/TofFrame` | R1; 64 native zone, m, 약 15Hz |
| `proximity_guard`, `desk_approach` | 구독 `tof/frame` | `cleany_interfaces/msg/TofFrame` | R1; 무효 zone은 자유 공간이 아님 |
| 두 제어 노드 | 구독 `odom` | `nav_msgs/msg/Odometry` | Sensor; 실제 속도와 정지 근거 |
| `proximity_guard` | 구독 `proximity/input`, 발행 `cmd_vel` | `geometry_msgs/msg/Twist` | R1; 최종 출력 30Hz, steady clock |
| `proximity_guard` | 발행 `proximity/status` | `std_msgs/msg/String` | R1; 감속과 정지 이유 |
| `proximity_guard` | 발행 `system/component_health` | `cleany_interfaces/msg/ComponentHealth` | R10; `source=tof_proximity`, 30Hz; 장애물 존재와 가용성을 분리 |
| `desk_approach` | 구독 `nav2/cmd_vel`, 발행 `motion/cmd_vel` | `geometry_msgs/msg/Twist` | R1; 명령 선택, 출력 30Hz |
| `desk_approach` | 구독 `navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | Action status QoS; 활성 Nav2 goal과 접근의 동시 소유 거부 |
| 두 제어 노드 | 구독 `system/heartbeat`, `system/state` | `SystemHeartbeat`, `SystemState` | R1 / R/T1; epoch, ROS stamp와 로컬 수신 시각 검사 |
| `desk_approach` | 제공 `desk/approach` | `cleany_interfaces/action/DeskApproach` | Action 기본값; 취소와 feedback 지원 |
| Mission Runtime | 호출 `desk/approach` | 같은 타입 | `desk_approach_enabled`일 때 Nav2 staging 도착 검증 이후 좌석 이동에만 호출; home은 생략 |
| Mission Runtime | 호출 `desk/approach/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | 재시작 시 journal의 이전 goal 취소와 정지 확인 |

`TofFrame.header.frame_id=tof_link`이며 배열은 펌웨어 순서다. TF를 발행하지 않는다.
장착 위치와 회전은 `tof.yaml`의 `sensor_*`가 소유하며 두 제어 노드에 함께 적용한다.
센서 +x는 전방, +y는 영상 왼쪽, +z는 위쪽으로 투영하며 flip은 실측해야 한다.

Action 목표는 base_link 원점에서 책상 평면의 y=0 교점까지 전방 거리(m)다.
범퍼 간격이 아니다. 회전 중에는 전진하지 않는다. 거리와 yaw가 허용 범위에 들면
0을 내보내고 신선한 odometry에서 정지가 `settle_s` 동안 유지되어야 성공한다.
Mission은 성공 이후 자신의 정지 확인도 통과해야 후속 작업을 시작한다.

Action goal은 기존 Mission lease의 mission_id, lease_id, epoch와 version을 전달한다.
서버는 권한을 발급하거나 연장하지 않는다. 상태나 heartbeat가 0.3초 이상 오래되거나
lease가 회수되면 중단한다. `require_supervision=false`는 독립 합성 시험용이다.

## 데이터와 제동 계약

제공된 VL53L7CH ESP32 JSONL `frame`을 사용한다. 연결 시 `stop` 응답 뒤 `cnh off`,
`config 64 15 400000`, `start`로 새 세션을 만든다. 재부팅 기본값인 10Hz는 수락하지
않는다. 이 드라이버는 펌웨어를 flash하거나 vendor 코드를 복제하지 않는다.

부분 USB 읽기를 모으며 배열 길이, 유효 플래그, status 5/9, target 수, 유한 거리와
sigma, 증가 sequence/device timestamp를 검사한다. 파싱 오류, USB backlog, replay와
장치 재시작은 포트를 닫고 새 세션으로 전환한다. 누락 frame을 대체하지 않는다.
첫 fresh session의 host/device offset으로 후속 지연을 감시한다. read 완료 시각에서
I2C read 시간과 한 측정 주기를 빼 보수적으로 stamp를 만든다. 노출 시각 동기화가
아니며, 첫 프레임부터 일정한 USB 지연은 완전히 관측할 수 없다. 호스트 부하와
센서 파이프라인 지연은 별도 실측해야 한다.

가드는 모든 유효 return을 base 직사각형 외곽과 비교한다. 단일 근거리 return도
정지에 기여하며, fit에서 제외한 zone도 장애물 검사에는 남는다. margin과 최대 허용
sigma의 3배를 여유로 빼고 다음 식으로 허용 접근 속도를 구한다.

```text
clearance >= closing_speed × (reaction_s + sample_age) + closing_speed² / (2 × deceleration)
```

명령 방향과 return 위치를 비교하며 접근 병진 성분은 속도 norm으로 보수적으로 계산한다.
회전에는 base 외접반경 × |yaw rate|를 더한다. 실제 odometry의 접근 속도가 제동
범위를 넘으면 0을 출력하고, 그 외에는 x, y, yaw에 같은 비율을 곱한다.
실제 제동은 base driver와 MCU의 책임이며 실측 감속도의 하한이 있어야 식이 의미가 있다.

명령, ToF, odometry가 오래되거나 필수 zone 무효, 캘리브레이션 미확인이면 0을 출력한다.
프로세스 종료는 기존 base/MCU watchdog이 처리한다. launch는 최종 `cmd_vel` 발행자를
하나로 구성하고 충돌을 감시한다. graph 감시는 다른 ROS 노드의 직접 발행을 차단하는
인증 수단은 아니다.

## 관측 한계

8×8 zone 중심을 pinhole 근사로 투영하고 선택 행의 column median으로 전면을 fit한다.
6개 이상 column, 최소 폭과 전체 선택 sample의 residual 조건을 요구한다. 8×8보다
작은 물체, 반사면, 투명체, blind spot의 부재를 증명하지 못한다. 평면 하나로 책상의
좌우 중심을 관측할 수 없다. Nav2 staging pose가 좌우 위치를 공급하고 ToF는 거리와
yaw만 보정한다. 측후방은 기존 LiDAR와 RGB-D에 의존한다. e-stop의 대체품이 아니다.

배포 설정은 `calibrated=false`, 필수 zone 64개로 출발한다. 제공된 15Hz 기록은 평균
유효 zone 약 34개이므로 기본 설정으로 주행을 허용하지 않는다. 장착 방향, 위치,
FOV, 반사 특성, 보호 zone과 실제 제동을 검증한 별도 프로파일을 사용한다.
무효 zone을 가리기 위해 근거 없이 `required_zones`를 줄이지 않는다.

## Nav2 재사용 범위

Humble [Collision Monitor 구현](https://api.nav2.org/nav2-humble/html/collision__monitor__node_8cpp_source.html)은
기존 LiDAR/depth stop과 slowdown을 계속 담당한다. 센서별 품질, acquisition 지연과
제동 검사는 최종 ToF 가드에 둔다. [Nav2 Docking Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_docking_server/)는
후보지만 현재 설치된 Humble 환경에는 없으므로 작은 접근 Action을 사용한다. Nav2
planning, recovery와 staging 도착 검증은 기존 구현을 재사용한다.
