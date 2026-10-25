# ToF와 정밀 접근

## 명령과 책임

`tof_enabled:=true`인 공통 navigation launch의 경로는 다음과 같다.

```text
Nav2 → nav2/cmd_vel → desk_approach 명령 선택 → motion/cmd_vel
  → 기존 pan gate → Collision Monitor → 기존 voxel/reactive guard
  → proximity/input → proximity_guard → cmd_vel → base driver → MCU watchdog

RGB-D 원본 + CameraInfo + TF + odom + 선택 명령
  → 기존 depth_coverage의 ToF 영역 평가 → tof/depth_coverage → proximity_guard
```

기존 안전 단계를 우회하거나 polygon을 줄이지 않는다. LiDAR와 Nav2는 경로와 관측한
장애물을, 기존 3D 안전 경로는 로봇의 보호 형상을 담당한다. 2D LiDAR가 정상이라는 사실만으로
ToF 높이의 책상 전면이나 돌출물까지 보호된다고 간주하지 않는다. ToF 미검출을 costmap의
자유 공간, clearing ray 또는 무한 거리로 발행하지 않는다.

`tof_driver`는 통신과 센서 세션, `proximity_guard`는 순간 제한, `desk_approach`는 명령
선택과 접근을 소유한다. Supervisor는 시스템 가용성, Fault, lease와 Mission 중단을 소유한다.
일반 장애물이나 정상 미검출 자체는 System Fault가 아니다. `tof_required=true`이면 가드의
`tof_proximity` 가용성을 `BASE_MOTION`의 필수 근거로 사용한다. 가용성과 특정 이동의
관측 가능성은 별개이며, 관측이 불충분한 이동은 로컬에서 정지한다.

## Zone과 프레임의 의미

[ST UM3183 Rev 7](https://www.st.com/resource/en/user_manual/um3183-a-guide-for-using-the-artificial-intelligence-enabler-vl53l7ch-and-vl53l8ch-multizone-timeofflight-tof-sensors-stmicroelectronics.pdf)의
6.5 target status와 6.6 driver 오류는 다른 필드와 의미다. 제공 firmware의 `valid`는 target이
있고 status 5/9인 경우에만 true이며 이 wire 계약은 유지한다.

| 분류 | 조건 | 처리 |
|---|---|---|
| `RANGE` | count > 0, status 5/9, valid=true, 양수 유한 거리와 sigma 한계 충족 | 장애물 검사와 평면 fit에 사용 |
| `NO_RETURN` | count=0, target_status=255, valid=false | 정상 수신된 미검출; 거리와 sigma 슬롯은 사용하지 않음. 빈 공간의 증거가 아님 |
| `UNRELIABLE` | 품질 불량, 허용하지 않은/알 수 없는 status, 범위 밖 거리나 불일치 | 위험 방향이면 정지. RGB-D 근거로 이 분류를 우회하지 않음 |
| `NOT_UPDATED` | target_status=0 | 미갱신; 위험 방향이면 정지 |
| 센서 세션 불가용 | sensor_ready=false, 통신/JSON/순서/지연 오류, firmware fault, 오래된 입력 | 모든 방향에서 fail-safe 정지 |

미검출의 잔여 거리 슬롯에는 실제 로그에서도 338mm 같은 값이 있다. 이를 가까운 물체나
먼 거리 어느 쪽으로도 사용하지 않는다. 반대로 driver event의 `status=255`는 zone의
`target_status=255`와 다르며 세션 불가용으로 처리한다. 3.5m 부근의 미검출 관찰은 모든
표면과 조명에서 그 거리까지 자유 공간을 보장하지 않는다. `max_range_m`도 검출 보장이 아니다.

fresh `sensor_ready=true` frame에 RANGE 또는 NO_RETURN이 있으면 수신된 관측으로 유지한다.
모든 Zone이 불신뢰/미갱신이면 관측 불가용이다. 일부 불신뢰 Zone은 개수로 평균내어 지우지 않고
방향별 판단에 남긴다. `ComponentHealth`는 가드와 입력의 가용성을 보고하며 주행 허용을 의미하지 않는다.

## 방향별 Coverage

전역 `min_valid`와 `required_zones`는 제거했다. 예전 프로파일에 이 키가 남으면 설정 오류로
거부하며, 값만 줄여 통과시키는 방식은 지원하지 않는다.

명령과 실제 odometry 각각에서 접근하는 축 방향과 Zone의 네 모서리 ray를 비교한다.
회전은 모든 Zone을 관련 영역으로 본다. 센서가 footprint 밖에 있을 때는 그 leading face도
보수적으로 포함한다. 반대 방향 Zone을 무시해도 그 곳을 자유 공간으로 선언하는 것은 아니며,
측후방 보호는 기존 안전 계층의 책임이다. 모든 유효 거리 return은 관련 Zone 분류와 별개로
근접 정지와 제동 검사에 계속 사용한다.

| 위험 방향의 관측 | 결과 |
|---|---|
| 신뢰할 수 있는 거리만 있음 | 기존 거리·속도 기반 제한 |
| 불신뢰/미갱신 Zone이 있음 | `UNRELIABLE_COVERAGE`, 정지 |
| 정상 미검출이 있고 독립 coverage 근거가 없음 | `NO_RETURN_UNCOVERED`, 정지 |
| 정상 미검출이며 일치하는 fresh RGB-D 근거가 있음 | `NO_RETURN_DEPTH_COVERED`, 제한 속도로 진행 가능 |

`depth_coverage`는 기존 finite depth 투영과 body sweep 로직을 재사용한다. ToF frustum과
교차하는 제동 영역의 cube 전체를 투영하며, 그 사각형의 모든 pixel에 유한하고 더 먼 depth가
있어야 한다. 희박한 point cloud, +inf, 시야각만의 겹침, 오래된 occupancy, guard의 PASS
문자열은 근거가 아니다. 현재 몸체 내부의 실제 형상만 제외하고, 보수적으로 확대한 ToF
footprint를 몸체 내부로 간주해 누락 영역을 없애지 않는다.

별도 `tof_sweep_v1` 근거는 같은 ToF 설정의 SHA256 `model_id`, 평가한 requested/measured
벡터, source age, 발행 시각과 covered/reason을 갖는다. 기존 `head_pan/coverage` 전체 몸체
진단과 구분한다. 가드는 ROS 시각과 monotonic 수신 나이를 모두 확인하고, 모델이 다르거나
현재 명령/실제 속도가 평가한 제동 궤적의 같은 방향 비례 축소로 포함되지 않으면 거부한다.
이 근거는 `source_timeout_s` 안에서만 유효하고 publisher도 하나여야 한다. 근거 생성 시
별도의 보수적인 감속도, 추가 지연과 frustum 이동 여유를 적용한다. 다른 방향이나 더 빠른
속도로 바뀌면 새 근거를 기다리며, 유한 depth가 사라지면 이전 근거도 철회된다.

미검출 보조 보호의 후보 상한은 0.08m/s와 0.10rad/s다. x/y/yaw를 같은 비율로 줄인다.
일반 제동은 margin과 최대 sigma의 3배를 빼고 다음 조건을 검사하며 실제 속도가 이미
허용 제동 범위를 초과하면 0을 출력한다.

```text
clearance >= closing_speed × (reaction_s + sample_age) + closing_speed² / (2 × deceleration)
```

RGB-D 근거는 정적 환경, 설정된 parked body와 기존 비례 제동 모델의 관측 증거다.
미장착 센서의 실제 중첩, 미세 물체, 움직이는 장애물, slip 또는 독립 축 제동의 보장은 아니다.
센서 오류나 품질 불량을 이 근거로 통과시키지 않는다.

## ROS endpoint

| 노드 | 방향과 기본 이름 | 타입 | QoS와 조건 |
|---|---|---|---|
| `tof_driver` | 발행 `tof/frame` | `cleany_interfaces/msg/TofFrame` | R1; 64 native Zone, 거리/sigma m, 약 15Hz; fault 시 sensor_ready=false |
| 두 ToF 제어 노드 | 구독 `tof/frame`, `odom` | `TofFrame`, `nav_msgs/msg/Odometry` | R1 / Sensor; frame_id=tof_link, canonical odom |
| `depth_coverage` | 발행 `tof/depth_coverage` | `std_msgs/msg/String` | R1; ToF 활성일 때 기본 20Hz, bounded JSON 근거 |
| `proximity_guard` | 구독 `tof/depth_coverage` | 같은 타입 | R1; 지연/모델/속도 불일치 거부 |
| `proximity_guard` | 구독 `proximity/input`, 발행 `cmd_vel` | `geometry_msgs/msg/Twist` | R1; 출력 30Hz steady timer |
| `proximity_guard` | 발행 `proximity/status` | `std_msgs/msg/String` | R1; 로컬 정지/제한 사유 |
| `proximity_guard` | 발행 `system/component_health` | `cleany_interfaces/msg/ComponentHealth` | R10; source=tof_proximity, detail JSON에 분류별 개수 |
| `desk_approach` | 구독 `nav2/cmd_vel`, 발행 `motion/cmd_vel` | `geometry_msgs/msg/Twist` | R1; 명령 선택, 출력 30Hz |
| `desk_approach` | 구독 `navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | Action status; 활성 Nav2와 동시 소유 거부 |
| 두 제어 노드 | 구독 `system/heartbeat`, `system/state` | `SystemHeartbeat`, `SystemState` | R1 / R/T1; epoch, stamp, 수신 시각 검사 |
| `desk_approach` | 제공 `desk/approach` | `cleany_interfaces/action/DeskApproach` | Action 기본값; feedback/cancel/result |
| Mission | 호출 `desk/approach`, 해당 cancel service | `DeskApproach`, `action_msgs/srv/CancelGoal` | 좌석 staging 이후; 재시작 시 journal goal 취소 |

`tof.yaml`의 공통 `/**` 설정을 보조 coverage와 두 ToF 제어 노드에 동일하게 적용한다.
개별 노드의 기하/제동 override가 공통 파일과 달라지면 보조 근거를 수락하지 않는다.
`tof_link` +x는 센서 전방, +y는 영상 왼쪽, +z는 위쪽으로 투영하며 실제 flip/extrinsic은
장착 후 확인해야 한다. 별도 TF를 발행하지 않는다.

## USB와 Desk Approach

USB는 `stop` 확인 후 `cnh off`, `config 64 15 400000`, `start`로 새 세션을 만든다.
재부팅 기본값 10Hz는 수락하지 않는다. 부분 read, 배열/valid 계약, sequence/device timestamp,
backlog와 지연을 검사한다. firmware event 오류와 비정상 state는 즉시 ROS sensor_ready를
철회하며 포트를 닫는다. exposure 시각은 동기화되어 있지 않으므로 read 완료에서 read 시간과
한 주기를 빼 acquisition을 추정한다. 첫 frame부터 일정한 USB 지연은 별도 실측해야 한다.

Desk Approach는 NO_RETURN을 평면점으로 채우지 않는다. 선택 행에 불신뢰/미갱신이 있으면
거부하고, 미검출을 제외한 실제 거리로 6개 이상 column(각 2개 이상 행), 폭과 residual 조건을
만족해야 한다. 보조 RGB-D coverage가 있어도 책상 평면 자체가 없으면 시작/성공할 수 없다.

Nav2 도착과 정지 이후 저속 거리/yaw 보정을 수행한다. 목표는 base_link 원점에서 평면의
y=0 교점까지의 거리이며 범퍼 gap이 아니다. 목표 위치의 fresh odometry 정지가 settle 동안
유지되어야 성공하고 Mission도 정지 피드백을 따로 확인한다. lease 식별자와 lifetime을 검증하며
권한 발급/연장, 자동 재개는 하지 않는다. 좌우 중심은 여전히 staging pose에 의존한다.

센서는 현재 단독 측정만 완료했고 로봇에는 미장착이다. `calibrated=false`와 독립 하드웨어
안전 근거 요구를 유지한다. 장착, 보호 높이와 RGB-D 중첩, 제동·지연, 표면 특성, 실제 접근
정확도를 검증하기 전 이 후보 설정으로 주행 가능하다고 해석하지 않는다.
