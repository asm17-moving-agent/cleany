# ToF 근접 감지와 책상 접근

ToF는 유효한 거리 return으로 근거리 장애물을 보완 감지하고 책상 전면까지의 거리와
방향을 측정한다. 정상 `NO_RETURN`은 장애물 미검출이며 주행 차단 사유가 아니다.
모든 Zone의 유효성이나 RGB-D free-space 증명을 요구하지 않는다. 미검출이 실제 빈
공간을 보장하지는 않으며 ToF와 다른 센서가 같은 장애물을 모두 놓칠 수 있다.

## 명령 경로와 책임

```text
Nav2 / 공식 teleop → nav2/cmd_vel → desk_approach 명령 선택 → motion/cmd_vel
  → 기존 pan gate → Collision Monitor → 기존 voxel/reactive guard
  → proximity/input → proximity_guard → cmd_vel → base driver → MCU watchdog

USB JSONL → tof_driver → tof/frame → 공통 Observation → guard와 approach
Mission → Nav2 staging와 정지 확인 → desk/approach → 후속 작업
```

기존 LiDAR, RGB-D와 Nav2 Collision Monitor의 보호, 최종 속도 경계와 모터 watchdog을
유지한다. ToF 미검출을 costmap clearing ray로 발행하지 않는다. 기존 head-pan
`depth_coverage` observer는 원래 설정과 책임을 유지하며 ToF에 의존하지 않는다.
ToF 전용 `tof/depth_coverage`, 모델 hash와 sweep proof 경로는 제거했다.

드라이버는 통신과 센서 세션을, guard는 순간 속도 제한을, approach는 명령 선택과
접근 Action을 소유한다. Supervisor는 가용성, Fault, lease와 Mission stop을 관리한다.
정상 미검출과 실제 장애물에 따른 정지는 System Fault가 아니다. `tof_required=true`는
`tof_proximity`를 기존 `BASE_MOTION`의 필수 가용성 근거에 추가한다. 센서 복구만으로
Mission을 자동 재개하지 않는다.

## 측정과 장애 구분

| 상태 | 조건 | 근접 감지와 평면 처리 |
|---|---|---|
| `RANGE` | count>0, status5/9, firmware valid=true, 유한 양수 거리와 허용 sigma | ROI 안의 실제 거리점으로 사용 |
| `NO_RETURN` | count=0, target_status255, valid=false | 정상 미검출. 잔여 거리/sigma 슬롯은 사용하지 않음 |
| `UNRELIABLE` | 불량 품질, 허용하지 않은 상태, 범위 위반 또는 불일치 | 해당 Zone 제외. 다른 Zone까지 무효화하지 않음 |
| `NOT_UPDATED` | target_status0 | 해당 Zone 제외 |
| 세션/입력 불가용 | sensor_ready=false, 통신/형식 오류, stale frame/odom, 전부 불신뢰/미갱신 | zero와 가용성 철회 |

전체 Zone이 NO_RETURN이어도 정상 관측이다. 전부 품질 불량인 frame은 미검출과 구별해
사용 불가능으로 처리한다. 개별 불량 Zone은 무시하므로 해당 방향의 검출 능력은 줄어든다.
`ComponentHealth`는 입력과 guard의 가용성이지 공간 전체의 안전 증명이 아니다.

firmware의 valid(status5/9) 의미는 바꾸지 않는다. Zone target_status255와 firmware
오류 event status255는 별개다. 후자는 sensor_ready를 즉시 false로 발행하고 세션을
닫는다. 통신 순서, backlog, acquisition 추정과 재연결 검사를 유지한다. 연결 시
`stop` 확인 후 `cnh off`, `config 64 15 400000`, `start`를 사용한다. 부분 read를 누적하며
재부팅 기본 10Hz frame은 주행 입력으로 수락하지 않는다. read 완료에서 read 시간과
한 주기를 빼 stamp를 추정하므로 첫 frame부터 존재한 일정 USB 지연은 실측이 필요하다.

## ROI 장애물 필터와 속도 제한

`guard_rows`와 `guard_columns`는 native 8×8 인덱스의 감시 ROI다. 기본은 전체 Zone이며
장착 후 바닥, 로봇 몸체와 필요한 보호 높이를 보고 조정한다. flip과 extrinsic은 거리
투영에 적용한다. 센서 +x는 전방, +y는 영상 왼쪽, +z는 위쪽이다. 별도 TF를 발행하지 않는다.

거리 임계값은 raw sensor range가 아니라 직사각형 footprint까지의 평면 거리에서
`margin + 3*max_sigma_m`를 뺀 clearance다. 높이는 ROI로 선택하며 3D 몸체 충돌 모델을
새로 만들지 않는다. 다음 값은 미장착 상태의 조정 가능한 초기 후보다.

| 설정 | 기본 후보 | 의미 |
|---|---|---|
| `obstacle_clearance_m` | 0.75m | 일반 장애물 검출 범위 |
| `min_cluster_zones` / `cluster_distance_m` | 2 / 0.20m | 상하좌우 인접하고 실제 점 간 거리도 가까운 cluster |
| `confirm_s` | 0.12s | 같은 Zone의 공간 support 지속 시간, 15Hz에서 보통 3 frames |
| `emergency_clearance_m` | 0.03m | 유효한 단일 Zone도 첫 frame부터 정지 |
| `hysteresis_m` / `release_s` | 0.05m / 0.20s | 검출 해제 거리와 시간 여유 |
| `max_speed` / `max_yaw_rate` | 0.25m/s / 0.3rad/s | 최종 명령 상한 |

guard의 30Hz timer가 같은 frame을 여러 번 확인한 것으로 세지 않는다. 수신한 새
frame마다 monotonic 시각으로 갱신하며 긴 입력 공백, 시간 역행 또는 불가용 입력이면
필터 이력을 폐기한다. 서로 떨어진 cluster나 반복 flash가 같은 Zone에서 지속된 것으로
누적되지 않는다. 확인된 Zone은 해제 거리 이내의 유효 return이 남으면 유지된다.

일부 확인된 Zone이 미검출되면 남은 실제 거리점을 계속 사용한다. 확인된 거리점이 모두
미검출되거나 해제 거리 바깥으로 사라지면 최대 release_s 동안
`OBSTACLE_HOLD`로 정지한다. 이동한 base frame에서 이전 점을 재사용하지 않는다.
비상 정지는 emergency_clearance+hysteresis 안의 return이 사라진 후 release_s 뒤 해제한다.
미검출만 있는 초기/정상 frame은 이 정지 이력을 새로 만들지 않는다.

확정된 거리점에 명령과 실제 odom의 접근 속도를 검사한다. x/y 중 접근하는 성분이
있으면 병진 norm을, 회전에는 footprint 외접반경을 사용한다. 허용 속도는 다음 식으로
구하고 x/y/yaw 전체를 같은 비율로 줄인다. 이미 실제 속도가 envelope를 넘으면 zero다.

```text
clearance >= closing_speed * (reaction_s + confirm_s + sample_age)
             + closing_speed^2 / (2 * deceleration)
```

`PROXIMITY_STOP`, `BRAKING_ENVELOPE`, `OBSTACLE_HOLD`는 장애물 대응이다. `LIMITED`는
감속, `CLEAR`는 검출에 의한 추가 제한 없음이다. 센서/odom/명령 지연, 미보정,
Supervisor 상실과 명령 graph 충돌은 별도의 정지 사유다. 매우 가까운 return과 해제
hold는 이동 방향에 관계없이 정지한다. 그 외에는 멀어지는 방향을 허용한다.

일반 검출의 지연, ROI 밖 물체, 기준보다 작은 cluster와 불량 return은 놓칠 수 있다.
제동 식에 confirm_s를 포함해도 그 지연과 센서 관측 한계의 물리 안전성을 보장하지 않는다.

## 책상 접근

Nav2 staging와 실제 정지 확인 후 `DeskApproach`를 요청한다. `approach_rows`(기본2–5)와
`approach_columns`(기본0–7)의 RANGE만 사용한다. 미검출/불량 Zone은 보간하지 않는다.
각 `min_plane_rows=2`개 이상의 실제 sample이 있는 `min_plane_columns=4`개 이상의
columns와 `min_plane_span_m=0.15m` 폭을 요구한다. column median으로
`x=distance+slope*y`를 fit하고 전체 선택 sample의 residual와 yaw 한계를 검사한다.
일부 불량 Zone만으로 실패하지 않지만 실제 support가 적거나 전면이 모호하면 시작을
거부하거나 진행 중 zero를 내고 abort한다. 미검출만으로 책상이나 도착을 추정하지 않는다.

정렬 후 최대0.04m/s, 0.08rad/s로 접근한다. 목표는 base_link 원점에서 평면 y=0 교점까지의
거리이며 범퍼 gap이 아니다. 좌우 중심은 staging pose에 의존한다. 목표 오차 내 실제
odom 정지가 settle_s=0.6초 유지되어야 성공한다. Mission도 정지 feedback을 확인한다.
취소, timeout, overshoot, no progress, Nav2 동시 goal, 센서/평면 소실과 lease 회수는
성공이 아니다. 서버는 기존 Mission lease의 epoch/version/lifetime만 검증하며 신규
권한을 발급하거나 연장하지 않는다. 재시작 journal의 해당 Action goal만 취소한다.

## ROS endpoint와 배포

| 노드 | 기본 endpoint | 타입과 조건 |
|---|---|---|
| tof_driver | 발행 `tof/frame` | TofFrame, R1, sensor_ready, 64 native Zone, tof_link, 거리/sigma m, 15Hz |
| guard / approach | 구독 `tof/frame`, `odom` | TofFrame R1 / Odometry Sensor, ROS와 monotonic freshness |
| proximity_guard | 구독 `proximity/input`, 발행 `cmd_vel` | Twist R1, 출력30Hz steady timer |
| proximity_guard | 발행 `proximity/status` | String R1, 제한/정지 사유 |
| proximity_guard | 발행 `system/component_health` | ComponentHealth R10, source=tof_proximity, 분류 개수와 pending/obstacle/emergency/held |
| desk_approach | 구독 `nav2/cmd_vel`, 발행 `motion/cmd_vel` | Twist R1, 단일 명령 선택 |
| desk_approach | 구독 `navigate_to_pose/_action/status` | GoalStatusArray, Action status QoS |
| guard / approach | 구독 `system/heartbeat`, `system/state` | 기존 메시지, R1 / R/T1, 시각과 epoch 검사 |
| desk_approach | 제공 `desk/approach` | DeskApproach Action, 기존 lease 식별자, feedback/result/cancel |

두 제어 노드에 동일한 ToF YAML을 적용하고 Mission과 기존 안전 경로의 canonical odom도
일치시킨다. navigation의 tof_enabled, Supervisor의 tof_required, Mission의
desk_approach_enabled는 별도 옵션이다. 기본은 opt-in과 calibrated=false다.
예전 min_valid/required_zones, no_return_max_*와 ToF coverage proof 설정은 제거한다.
TofFrame의 sensor_ready 계약은 유지되므로 외부 publisher도 정상 frame에 true를 설정한다.

아직 센서는 로봇에 미장착이다. FOV/extrinsic/ROI, 표면/조명별 검출, 실제 지연과 제동,
필터 오탐/미탐, 접근 정확도와 독립 정지를 장착 후 측정해야 한다. 기존 독립 안전 장치를
활성 상태로 유지하고 후보 설정을 물리 안전 검증 결과로 해석하지 않는다.
