# cleany_system_supervisor

Cleany의 시스템 가용성과 Mission 실행 허가를 관리하는 단일 ROS 2 Humble Python
node다. Supervisor는 실제 안전 정지 장치가 아니며 Mission 재개, Skill retry,
Nav2 BT Recovery와 Lifecycle Manager의 기능을 대신하지 않는다.

## 현재 구조와 설계

이 브랜치의 Mission Manager는 동기식 Navigator → Perception → Planner → Skill
port를 호출하는 순수 FSM이다. 기존 시스템 수준 health/fault manager나 Mission ROS
node는 없었다. Base의 diagnostics와 MCU watchdog, Nav2 lifecycle을 재사용한다.
기존 `E_STOP`/`HARDWARE_ERROR` ModuleResult는 작업 종료 정보로 유지하며, 일반
파지/IK/Nav2 실행 실패를 Supervisor Fault로 자동 변환하지 않는다.

```text
scan / pose / image / diagnostics / lifecycle / owner health / independent safety
                         ↓ HealthMonitoring (freshness + semantic validity)
                     FaultManager (concurrent faults + raise/clear hysteresis)
                         ↓ CapabilityManager (dependency graph)
                     Operating State (recovery state와 독립)
                         ↓ MissionPermissionManager → StopMission Action
                         ↓ RecoveryCoordinator → owner RecoverComponent Action
                                          ↓ fresh stable health verification
```

`models.py`, `ports.py`, `health.py`, `faults.py`, `capabilities.py`, `permissions.py`, `recovery.py`,
`supervisor.py`는 ROS 비의존 core다. `node.py`는 입력 검증, snapshot 발행, 서비스와
Action 서버를 연결하며 `ros_transport.py`가 비동기 Action client를 소유한다.
정책 시간은 monotonic, ROS stamp는 센서 데이터 유효성 검증에만 사용한다.
ROS 실행은 callback을 블로킹하지 않으며 Mission의 기존 동기 port는 별도 worker에서
실행한다. 하나의 복구만 실행해 컴포넌트 간 충돌을 보수적으로 방지한다.

## 운영 상태

| 상태 | 조건 |
|---|---|
| INITIALIZING | boot grace 중 필수 기능/안전 증거가 아직 검증되지 않음 |
| OPERATIONAL | 모든 Capability 및 필수 안전 증거가 fresh/stable, 운영 잠금 Fault 없음 |
| DEGRADED | 필수 기능과 안전은 검증됐지만 일부 비필수 Capability 제한 |
| FAULT_LOCKED | 운영 잠금 Fault, 필수 기능 불가, 또는 grace 이후 필수 정보 미확인 |

Severity만으로 상태를 결정하지 않는다. 기본 필수 Capability는 `BASE_MOTION`이며
정책에서 변경할 수 있다. 초기 및 stale 상태는 UNKNOWN이다. Capability 차단은
Fault 발생 debounce를 기다리지 않고 즉시 적용한다. 정상 복귀에는 `clear_s` 동안
연속 fresh 정상 보고가 필요하다. 복구 중/취소 미확인 컴포넌트는 계속 제한한다.

## 기본 Capability와 Fault

| Capability | 직접 증거 | 의존성 |
|---|---|---|
| BASE_MOTION | base diagnostics, 소유자의 arm_motion_safe 보고 | 없음 |
| NAVIGATION | 유효 scan, covariance 제한 내 pose + localization lifecycle, Nav2 lifecycle | BASE_MOTION |
| MANIPULATION | Arm 소유자의 semantic health | 없음 |
| VISUAL_PERCEPTION | 유효 image, Perception 소유자의 semantic health | 없음 |
| MOBILE_MANIPULATION | 없음 | NAVIGATION, MANIPULATION, VISUAL_PERCEPTION |

`arm_motion_safe`는 팔 통신만으로 이동 안전을 추정하지 않기 위한 별도 소유자 보고다.
카메라 frame 수신은 Perception 가용성을 대신하지 않는다. 허가는 특정 동작의 충돌,
파지 상태, 경로, 힘, 속도 안전 검사를 대체하지 않는다.

| Fault | 기본 처리 |
|---|---|
| BASE_UNAVAILABLE | base 오류/stale, CRITICAL, latched, 전체 잠금 |
| LIDAR_UNAVAILABLE | scan 중단/무효, Navigation 제한 |
| LOCALIZATION_UNAVAILABLE | pose stale/무효/covariance 초과/lifecycle 비활성, Navigation 제한 |
| NAVIGATION_UNAVAILABLE | 필수 Nav2 lifecycle 비활성/무응답, Navigation 제한 |
| CAMERA_UNAVAILABLE | image stale/무효, Visual Perception 제한 |
| PERCEPTION_UNAVAILABLE | 소유자 상태 불가/미확인, Visual Perception 제한 |
| ARM_UNAVAILABLE | 소유자 상태 불가/미확인, Manipulation 제한 |
| ARM_MOTION_UNSAFE | 이동 안전 상태 불가/미확인, Base Motion 제한 → 기본 전체 잠금 |
| SAFETY_UNVERIFIED | 독립 보호 계층 불가/stale, CRITICAL, latched, 전체 잠금 |
| ESTOP | engagement 또는 물리 해제 확인 불가/stale, CRITICAL, latched, 전체 잠금 |
| MISSION_STOP_FAILED:<lease_id> | 중단 실패/timeout, CRITICAL, 확인된 stop 결과 전까지 잠금 |

Fault는 code/source/severity/영향 Capability/발생 및 최종 관측 monotonic 시각/
active/latched/recovery policy/detail을 유지한다. latched Fault reset은 해당 원인의
fresh/stable 정상 보고가 있어야 한다. E-stop reset 서비스는 하드웨어 해제를
수행하지 않는다. 물리 해제 및 보호 계층 상태가 모두 정상이어야 허가가 가능하다.
여러 Fault 중 하나만 reset해도 다른 잠금 Fault는 유지된다.

## ROS 인터페이스

이름은 상대 이름이며 root namespace 기준 아래와 같다. 공유 type은
`cleany_interfaces`에서 생성한다.

| 이름 | 타입 | 계약 |
|---|---|---|
| /system/state | SystemState | epoch, revision, 운영 상태, 전체 MissionPermission snapshot |
| /system/capabilities | SystemCapabilities | 전체 기능 가용성 |
| /system/faults | SystemFaults | 활성 및 해제 Fault snapshot |
| /system/recoveries | SystemRecoveries | job별 state/attempt/검증/점유 상태 |
| /system/heartbeat | SystemHeartbeat | 프로세스 epoch/revision과 timeout 안내 |
| /system/check_permission | CheckPermission service | 허가 취득/갱신/완료 후 반환 |
| /system/request_fault_reset | RequestFaultReset service | 원인 해소 검증 후 latch 해제 |
| /system/request_recovery | RequestRecovery action | 컴포넌트 복구 조정 및 health 재검증 완료 |
| /mission/stop | StopMission action | Mission 소유자 중단; 수락과 stopped 결과 구분 |
| <owner>/recover | RecoverComponent action | 컴포넌트가 실제 복구를 수행 |
| /system/component_health | ComponentHealth | 설정된 소유자의 semantic 상태만 수락 |
| /safety/status | SafetyStatus | 독립 안전 adapter의 보호, E-stop, 물리 해제 증거 |

상태 4개 topic은 reliable / transient local / keep last 1, 20 Hz snapshot이다.
heartbeat는 reliable / volatile / depth 1, 20 Hz다. 센서 입력은 sensor-data QoS,
health/diagnostics/safety 입력은 reliable volatile이다. old retained 상태는 허가가
아니다. client는 heartbeat freshness, epoch, lease expiry를 별도로 검사해야 한다.
revision은 snapshot 순서이며 lease version은 개별 취득 세대를 나타낸다.

기본 ROS 입력은 설정 파일의 `ros`에서 관리한다. LiDAR는 fresh stamp, frame,
geometry, 정상 beam 비율을 검사한다(+inf no-return 허용). Localization은 fresh pose,
유한 값, quaternion, covariance와 lifecycle active를 함께 요구한다. Nav2의 설정된
모든 lifecycle node가 active 및 응답 중이어야 한다. image는 stamp와 payload를
검사한다. Base는 `base_driver` diagnostics의 level과 feedback age를 확인한다.
이 검사는 Localization 정확도, LiDAR 환경 인지 품질이나 물리 정지 성능의 증명은 아니다.

## Mission 실행 허가와 중단

1. client가 새 heartbeat로 epoch를 확인하고, Mission Manager가 환경/작업을 검증한다.
2. `CheckPermission`에 epoch, mission_id, 필요한 전체 Capability 집합을 보낸다.
   `lease_id=''`는 새 허가 취득이다. 알 수 없거나 빈 capability 집합은 거부한다.
3. 반환된 lease_id/version/epoch와 짧은 TTL(기본 1.5초)을 저장하고 주기 갱신한다.
   갱신은 같은 mission/token/version/Capability 집합과 미만료 조건을 요구한다.
4. Capability 상실/운영 잠금/TTL 만료 시 취소 불가능한 중단 Action을 요청한다.
   Action 수락은 stop proof가 아니다. 실패/3초 timeout은 전체 잠금으로 확대한다.
5. Mission이 작업 및 물리 환경을 재검증하고 명시적으로 새 허가를 얻어야 재개한다.
   기존 scene/plan은 재사용하지 않는다. Supervisor는 재개를 호출하지 않는다.

Supervisor 종료 시 갱신이 끊기며 Mission client가 자신의 monotonic TTL 및 heartbeat
timeout으로 차단한다. 재시작은 새 epoch와 모든 UNKNOWN 상태에서 시작한다.
이전 lease를 갱신할 수 없다. Mission 종료 후 `release=true`는 완료 확인을 소유하는
Mission Manager만 보내야 하며, 단순 취소 수락 후 보내면 안 된다.

Mission Manager의 `SupervisedMissionNode(manager)`는 명시적 `stop_controller`가 있는
실제 adapter를 주입받는다. `submit()`이 기존 FSM을 worker에서 실행하며 독립 timer가
허가 만료/철회를 감시한다. 무감독 기존 순수 core 호출은 기존 mock 테스트 호환을 위해
유지한다. 실제 자동 운영에는 이 bridge 또는 동일 계약을 지키는 adapter가 필요하다.
실제 Mission/Arm/Nav2 port의 production wiring이나 Dashboard 접수 API는 만들지 않는다.

## 복구 정책

정책 원본은 [`config/system_supervisor.yaml`](config/system_supervisor.yaml)이며
package share의 `config/`로 설치한다. dependency graph cycle, 미지 source, 비유한
deadline 등의 설정은 시작 시 거부한다.

기본 owner endpoint는 `lidar/recover`, `localization/recover`, `navigation/recover`,
`camera/recover`, `arm/recover`다. 현재 해당 실물 owner Action 구현이 없으므로 기본
자동 복구는 비활성이다. 수동 요청도 endpoint 부재를 실패로 반환하며 재시도 예산을
소모한다. owner를 구현하고 검증한 뒤 `automatic: true`로 설정한다. Nav2 내부의
일반 BT Recovery를 추가 호출하거나 노드를 shell 명령으로 재시작하지 않는다.

job 상태는 IDLE/RUNNING/SUCCEEDED/FAILED/CANCELED이며 운영 상태와 독립이다.
기본 max_attempts=2, attempt=5초, total=20초, health verification=3초,
cancel confirmation=2초다. 우선순위는 CRITICAL → ERROR → WARNING이며 실행 중인
복구를 다른 복구와 겹치지 않는다. 안전 Fault 발생/미확인 Mission 중단 시 복구를
취소한다. 복구 시작에는 독립 안전 adapter의 whole-robot `stopped` 증거도 필요하다.
이는 복구 시작 조건이며, 승인된 복구 자체가 수행하는 움직임을 Fault로 취급하지 않는다.
운영 잠금 상태에서도 보호 계층과 정지 조건이 검증되면 원인 컴포넌트의 복구를 수행할 수 있다.
취소 미확인 시 FAILED로 보고하되 실행 자원을 계속 점유하여 추가 복구를
차단한다. terminal result가 뒤늦게 확인되어야 점유를 해제한다. 같은 Fault episode의
반복 요청은 같은 job을 반환하므로 무제한 재시도 우회가 불가능하다.

owner success 후에도 결과 시각 이후 새 정상 health와 안정화 시간이 필요하다.
미해결 health, timeout, 취소 미확인은 Fault와 기능 제한을 해제하지 않는다.
Mission stop timeout 뒤에는 해당 `MISSION_STOP_FAILED:<lease_id>` reset 요청으로
동일 stop Action을 다시 조회한다. 서비스 응답은 확인 요청일 뿐이며, 새 stopped 결과를
받아야 잠금 원인이 해소된다. owner는 Supervisor 재시작을 포함해 충돌하는 recovery
goal을 거부해야 한다. Supervisor 메모리의 전역 직렬화만으로 원격 owner의 단일 실행을
보장한다고 가정하지 않는다.

## 실행 및 검증

레포 루트에서(호스트에서는 기존 Humble Distrobox 경로 자동 사용):

```bash
make test-supervisor-core
make build-supervisor
make test-supervisor
```

Humble 환경에서:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_system_supervisor system_supervisor.launch.py
```

기본 profile은 실물 안전/Arm owner의 증거가 없으면 잠긴다. 실물 연결 없이 자동
운영을 보려면 테스트 graph의 명시적 synthetic peer를 사용한다. production profile을
우회하거나 하드웨어 정상 상태를 가정하는 실행 모드는 제공하지 않는다.

테스트는 pure core clock/ports와 ROS graph synthetic 센서/lifecycle/owner Actions로
구분한다. 기존 정상 FSM 회귀도 함께 실행한다. 실제 주행/조작/하드웨어 E-stop의
물리 검증 결과로 해석하면 안 된다.

## 현재 한계 및 후속 검증

- 실물 독립 E-stop/protection 상태 adapter, Arm health/arm_motion_safe adapter,
  Perception readiness publisher, 실제 component recovery Actions는 추가 구현이 필요하다.
- Base MCU의 기존 deadline/watchdog 코드는 재사용하며 ESP32 통신 두절에서 ROS stop
  전달 성공을 가정하지 않는다. 실제 통신 차단 후 PWM/모터 정지 시간은 별도 계측한다.
- Mission/하위 제어기 crash 뒤에도 물리 안전을 보장하려면 하위 permit/watchdog 또는
  독립 안전 회로가 필요하다. 현재 lease는 이 bridge를 사용하는 Mission 소프트웨어를
  제한하며 임의의 `/cmd_vel` publisher나 legacy Skill 실행기를 차단하지 않는다.
- 안전 보고 source의 인증/단일 publisher 강제, 장기 Fault/history retention, 하드웨어
  정지 센서, restart 후 물체 보유/팔 자세 재검증과 operator 절차는 추가 검증 대상이다.
- 전체 시스템 자동 기동 launch에는 연결하지 않았다. 제공 launch는 Supervisor만
  시작하며 센서/컴포넌트/독립 안전 소유권은 기존 launch에 남긴다.
- 실제 adapter가 동기 호출 중단을 지원하지 않으면 중단을 성공 처리하지 않으며
  timeout 및 운영 잠금으로 남는다.

## 이번 변경의 검증 기록

2026-10-16, `feat/system-supervisor`, 기준 commit `ba7ede1`에서 실행했다.

| 검사 | 결과 | 증거 범위 |
|---|---|---|
| `make test-supervisor-core` | 34 passed | Supervisor pure core 17 + Mission core/회귀 17 |
| `make test-supervisor` | 50 passed | interfaces 10 + Mission 17 + Supervisor 23(core 17, ROS graph 6) |
| 관련 package build | 3 packages 성공 | Humble IDL 생성, Python 설치, launch/config 설치 |
| 설치된 launch smoke | INITIALIZING → FAULT_LOCKED | 센서/안전 증거 없는 기본 profile의 보수적 기동 |

ROS graph는 격리된 domain 184에서 synthetic peer만 사용했다. launch smoke는 domain
185에서 수행했다. boot/degraded/locked, hysteresis와 동시 Fault, dependency 전파,
lease 취득/갱신/철회/만료/epoch, Supervisor 재시작과 종료, stop 수락과 실제 완료,
stop 실패와 재확인, recovery 성공/실패/timeout/재시도 예산/취소 미확인 격리,
안전 Fault에 의한 recovery 취소, 명시적인 Mission 재검증과 기존 정상 Mission을 검사했다.
ROS graph의 복구 성공과 stop proof는 synthetic 소유자 응답이며 실물 증거가 아니다.

전체 workspace build/test는 실행하지 않았다. 기존 perception/grasp wire 계약은 변경하지
않았으며 변경된 공통 interfaces와 Mission/Supervisor 3개 package의 build/test를 실행했다.
실제 Nav2 주행, Arm 동작, MCU disconnect 시 물리 정지, E-stop 회로 및 재기동 안전성은
검증하지 않았다. KB submodule, firmware, 기존 Nav2 Recovery는 변경하지 않았다.

## 변경 파일 전체 목록

경로는 레포 루트 기준이다. 생성 build/install/log/cache는 목록에 포함하지 않는다.

### 신규 파일

- `ros2_ws/src/cleany_system_supervisor/config/system_supervisor.yaml`
- `ros2_ws/src/cleany_interfaces/action/RecoverComponent.action`
- `ros2_ws/src/cleany_interfaces/action/RequestRecovery.action`
- `ros2_ws/src/cleany_interfaces/action/StopMission.action`
- `ros2_ws/src/cleany_interfaces/msg/ComponentHealth.msg`
- `ros2_ws/src/cleany_interfaces/msg/MissionPermission.msg`
- `ros2_ws/src/cleany_interfaces/msg/SafetyStatus.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemCapabilities.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemCapability.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemFault.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemFaults.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemHeartbeat.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemRecoveries.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemRecovery.msg`
- `ros2_ws/src/cleany_interfaces/msg/SystemState.msg`
- `ros2_ws/src/cleany_interfaces/srv/CheckPermission.srv`
- `ros2_ws/src/cleany_interfaces/srv/RequestFaultReset.srv`
- `ros2_ws/src/cleany_mission_manager/cleany_mission_manager/ros_supervision.py`
- `ros2_ws/src/cleany_mission_manager/tests/test_supervision.py`
- `ros2_ws/src/cleany_system_supervisor/README.md`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/__init__.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/capabilities.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/config.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/faults.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/health.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/models.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/node.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/permissions.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/ports.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/recovery.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/ros_transport.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/sensors.py`
- `ros2_ws/src/cleany_system_supervisor/cleany_system_supervisor/supervisor.py`
- `ros2_ws/src/cleany_system_supervisor/launch/system_supervisor.launch.py`
- `ros2_ws/src/cleany_system_supervisor/package.xml`
- `ros2_ws/src/cleany_system_supervisor/resource/cleany_system_supervisor`
- `ros2_ws/src/cleany_system_supervisor/setup.cfg`
- `ros2_ws/src/cleany_system_supervisor/setup.py`
- `ros2_ws/src/cleany_system_supervisor/test/test_core.py`
- `ros2_ws/src/cleany_system_supervisor/test/test_ros_graph.py`

### 수정 파일

- `Makefile`
- `ros2_ws/README.md`
- `ros2_ws/src/cleany_interfaces/CMakeLists.txt`
- `ros2_ws/src/cleany_interfaces/README.md`
- `ros2_ws/src/cleany_interfaces/package.xml`
- `ros2_ws/src/cleany_interfaces/test/test_interface_contract.py`
- `ros2_ws/src/cleany_mission_manager/README.md`
- `ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/manager.py`
- `ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/ports.py`
- `ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/result.py`
- `ros2_ws/src/cleany_mission_manager/cleany_mission_manager/core/states.py`
- `ros2_ws/src/cleany_mission_manager/package.xml`
