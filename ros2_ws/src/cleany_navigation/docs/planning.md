# 경로 계획과 주행 제어

기본 AMCL 및 Omni 프로필은 `nav2_smac_planner::SmacPlannerLattice`와
`cleany_navigation::MPPIController`를 사용한다. MPPI는 `Omni` 모델로 전후와 횡이동을 허용한다.
명시적인 `axis_motion:=true`는 기존 축 전용 DWB 프로필을 선택한다.

launch는 설치된 `config/lattice/omni-r10cm.json`을 사용하며,
`GridBased.lattice_filepath`를 명시하면 해당 경로를 우선한다.
이 lattice는 공식 Nav2 1.1.20 Generator의 5cm 격자, 10cm 반경 설정,
16개 heading과 144개 primitive를 사용한다. 반경 설정은 실물 최소 회전 반경 측정값이 아니다.

커스텀 MPPI와 critic은 `mppi/src` 및 `include/cleany_navigation/mppi`에서 빌드한다.
기본 Nav2 MPPI와 구분하도록 controller는 `cleany_navigation::MPPIController`,
critic은 `cleany_navigation::mppi::critics`로 등록한다.
원본 라이선스와 설명은 `mppi/LICENSE.md`와 `mppi/README.upstream.md`에 보존한다.

## 휴대폰 시연 프로필

`phone_navigation.launch.py`는 `phone_motion_profile:=longitudinal`을 기본으로 사용한다. 이 프로필은 통합 MPPI의 DiffDrive 모델과 직진/제자리 회전 Lattice primitive를 사용한다. `phone_motion_profile:=omni`는 기존 Omni 모델과 bundled Omni primitive를 사용한다. 두 프로필 모두 일반 navigation의 기본값을 변경하지 않는다. 이 launch는 명시적 robot_params_file과 기존 안전 센서 입력이 필요하다.

`omni`는 순정 `nav2_smac_planner::SmacPlannerLattice`만 전역 계획에 사용한다. 직선 우선 연결, 고정 방향 XY 탐색, 별도 몸체 방향 탐색과 거리 기준 방향 선택을 거치지 않는다. 현재 phone Omni 기본값인 `omni-r10cm-balanced.json`은 기존 144개 primitive의 좌표와 방향을 그대로 보존하고, 오른쪽 횡이동 16개의 잘못된 `trajectory_length`와 `straight_length`만 실제 거리로 교정한 파일이다. 원본 `omni-r10cm.json`은 일반 navigation 프로필용으로 보존한다.

| Lattice 설정 | 값 | 역할 |
| --- | --- | --- |
| `rotation_penalty` | 5.0 | 제자리 회전 비용 |
| `reverse_penalty` | 1.2 | 후진 확장 비용 |
| `non_straight_penalty` / `change_penalty` | 1.2 / 0.05 | 곡선과 회전 방향 전환 비용 |
| `cost_penalty` | 2.0 | 장애물 비용장 반영 |
| `max_planning_time` | 2.0s | 검색 시간 예산 |
| `tolerance` | 0.0m | 다른 목표 셀로 대체하지 않음 |
| `smooth_path` / analytic expansion | false / 0m | primitive 기반 경로 유지 |

순정 Humble Lattice는 클릭 좌표와 yaw를 5cm 셀 및 16개 방향으로 양자화한다. `tolerance=0`도 연속 좌표를 정확히 보존한다는 뜻은 아니다. 컨트롤러의 3cm/0.1rad 도착 판정은 반환된 경로 끝점 기준이다. 웹 UI는 미리보기 반환 끝점을 빨간 목표와 좌표/방향 숫자로 표시하고, 사용자가 이동을 누르면 그 자세를 전달한다. 조종기의 3cm/0.1rad 도착 검사도 이 표시 목표에 적용하며 허용오차를 늘리지 않는다.

MPPI는 `Omni`, 20Hz, 60×0.05s 예측과 1000개 샘플을 사용한다. X/Y 표준편차는 0.1m/s, 회전 표준편차는 0.2rad/s, temperature는 0.1이다. `PathAlignCritic`은 `use_path_orientations=false`로 Lattice 경로의 위치만 평가하며 가중치는 5다. 이동 중 몸체 방향은 방향별 이동 비용, 장애물과 회전 비용으로 결정한다. `PathAngleCritic`과 강제 회전 단계는 비활성이다. GoalCritic/PathFollowCritic의 전환 거리는 0.5m, 가중치는 40/30이다. GoalAngleCritic은 0.15m에서 가중치 12로 최종 방향을 평가한다. TwirlingCritic 가중치 1은 불필요한 회전에 비용을 준다. CostCritic의 일반 반발 비용을 생략하는 도착 영역은 0.3m이며 footprint 충돌 검사는 유지한다. 이 설정은 특정 거리에서 회전·전진·회전 순서를 강제하지 않는다.

LiDAR phone 시연의 footprint는 CAD 바퀴 포함 하부 외곽 0.477×0.6548m를 사용하고 `footprint_padding=0.05`를 적용한다. Collision Monitor 정지 사각형은 같은 외곽을 사방 5cm 확장한 0.577×0.7548m다. 이 여유는 카메라·팔 높이의 3D 충돌 검사나 실측 제동 거리 보증을 뜻하지 않는다.

휴대폰 Omni 자동주행은 병진 합성 속도 `hypot(vx, vy)`를 0.5m/s, 회전을 0.7rad/s로
제한한다. 주황 영역은 바퀴 외곽 사방 0.16m이며 병진 0.16m/s와 회전 0.3rad/s를
적용한다. 작은 요청은 키우지 않고, 혼합 명령이 경계를 넘으면 Collision Monitor가
Twist 전체를 같은 비율로 줄여 곡률을 유지한다. 도착점 0.2m 안에서는 병진 0.06m/s,
회전 0.15rad/s를 목표 상한으로 사용한다. 휠 10rad/s는 드라이버와 같은 최종 구동
상한이다. 병진과 회전을 섞으면 이 구동 상한으로 더 느려질 수 있다. 별도 종방향
프로필의 주행 속도는 기존 값을 유지한다.

횡이동 차단 프로필은 `StraightFirstPlanner`로 몸체 전체가 통과 가능한 직선을 먼저 검사하고, 불가능할 때 Smac Lattice 탐색으로 전환한다. 횡이동 경로는 사용하지 않으며 추가 검색 padding은 0으로 두고 기존 footprint의 5cm 여유를 그대로 검사한다.

longitudinal 프로필의 `GridBased.prefix_shortcut_distance=1.0`은 Lattice 경로 앞쪽 1m 안에서 연결 가능한 먼 경로점까지 직진과 필요한 제자리 회전으로 연결한다. 격자 시작점에 붙기 위한 짧은 이동과 반대 방향 회전을 줄이며 남은 경로와 최종 자세는 보존한다. 연결은 전체 footprint의 연속 sweep과 최신 costmap을 모두 통과해야 한다. 통과하지 못하면 원래 경로를 유지하며, 다른 프로필의 기본값 0은 이 처리를 끈다.

longitudinal의 `PathAngleCritic.respect_in_place_rotations=true`는 Lattice 경로의 제자리 회전을 평가한다. 로봇이 선두 경로점의 0.075m 안에 있고 같은 XY에서 yaw만 바뀌면 그 회전 구간의 마지막 yaw를 후보 궤적의 방향 목표로 사용한다. `path_rotate_enabled`는 방향 오차 0.1rad를 넘는 동안 후보 궤적과 출력의 목표 XY 속도를 0으로 제한한다. Omni의 가속도 제한이 켜져 있으면 현재 이동을 먼저 감속하며 그 감속 이동도 충돌 검사에 포함한다. 제한을 끈 기존 프로필은 즉시 XY=0을 적용한다. 측정된 초기 속도는 충돌 예측에 그대로 반영한다. 일반 이동 구간은 기존 경로 방위각 평가를 유지한다.

longitudinal은 위치가 목표의 3cm 안에 들어오면 최종 GoalAngleCritic을 우선한다. 최종 회전 XY 제한은 2.5cm에서 시작해 3cm 밖에서 해제된다. Omni는 이 강제 단계를 사용하지 않고 비용 평가와 물리적 속도·가속도 제약으로 명령을 정한다.

## 위치 정합과 복구

휴대폰 Omni의 `slip_recovery:=false`는 `slip_monitor`와 로그를 유지하고
`SlipRecovery` BT, 종방향 재시도 planner와 `traction_brake`를 실행하지 않는다.
충돌 감시 입력은 `motion/cmd_vel`이며 기본 Nav2 동작과 아래 위치 정합 복구는
계속 적용한다. 슬립 BT를 켜면 입력은 `motion/traction_cmd_vel`로 바뀐다.

휴대폰 LiDAR 시연의 `localization_guard`는 scan 시각의 base pose로 레이저 끝점을
지도에 투영하고 점유 셀까지의 거리를 평가한다. 지도 밖과 미확인 영역의 끝점은
일치로 세지 않는다. 상위 25% 잔차는 평균에서 제외하되 일치 비율의 분모에서는
제외하지 않는다. 따라서 일부 동적 물체를 허용하면서 미확인 영역만으로 정상 판정을
만들지 않는다. 공분산이 작다는 사실은 지도 정합 검사를 생략하는 근거가 아니다.
AMCL의 분산 계산에서 생기는 -1e-9 이내의 수치 오차는 0으로 해석하지만 더 큰
음수, 비유한 값과 설정 상한을 넘는 분산은 정상 위치 근거로 사용하지 않는다.

`phone_localization.yaml`의 실험 기준은 15cm 이내 끝점 30% 이상, 절사 평균 잔차
18cm 이하이며 최소 60개 유효 끝점을 요구한다. 1초 연속 정상 관측으로 검증된
위치만 available이 된다. 이 위치에서 가벼운 불일치가 생기면 degraded로 전환하고
AMCL이 계속 보정하는 동안 기존 목표와 허가를 유지한다. 새 초기 위치를 아직
검증하지 못했거나 센서·TF 입력이 오래됐으면 unknown이며 주행을 허용하지 않는다.
반복 heartbeat는 안정화 시간을 늘리지 않는다.
이 기준은 지도 품질과 실제 바닥에서 추가 검증해야 하는 시연용 초기값이다.

degraded는 정상 속도 상한의 30%를 요청한다. 현재 병진 0.15m/s와 회전 0.21rad/s이며
더 낮은 장애물·도착 상한이 우선한다. MPPI와 기존 Collision Monitor가 같은
`navigation/localization_speed_limit`을 소비한다. Monitor의 제한 하강 속도는 초당
30%p이며, 정상 복귀 시 MPPI의 기존 가속도 제한으로 회복한다. 제한 메시지가
0.6초 동안 없거나 0%를 받으면 Monitor는 즉시 0을 출력한다.

개선은 최고 기록보다 평균 오차가 2cm 줄거나 일치율이 5%p 증가한 경우다. 정상
기준을 만족하는 새 관측도 개선 시각을 갱신한다. 다만 1초 연속 정상 확인 전에는
감속을 유지하고, 짧은 정상 관측이 끼어도 총 8초의 대기 한도는 초기화하지 않는다.
4초간 개선이 없거나 불일치 총 시간이 8초에 도달하면 unavailable로 정지한다. 일치율 10%
미만과 평균 오차 35cm 초과가 동시에 0.8초 지속되면 먼저 정지한다. 잘못된 공분산,
큰 불확실성, 입력 단절은 이 대기 시간을 받지 않는다. 정상 기준을 다시 1초 유지하면
available이 되지만, 이미 취소된 목표를 자동 재개하지 않는다.

Supervisor가 위치 정합 Fault로 주행 허가를 회수하고 이전 목표의 정지를 확인하면
base_disarmed가 안정된 뒤 `localization/recover`를 요청한다. 이 node는 현재 pose
주변 반경 0.8m와 yaw ±0.7rad만 탐색한다. 별도 학습/검증 빔, 다른 위치 후보와의
점수 차이, 다음 신선한 scan을 확인하고 나서만 AMCL initialpose를 발행한다.
복구는 휠을 움직이지 않으며 지도도 수정하지 않는다.

복구 후보는 일치 비율 50% 이상, 기존 절사 평균 대비 4cm 이상 개선되어야 한다.
공분산만 높고 기존 pose의 정합은 정상인 경우에는 개선량 대신 같은 엄격한 일치
비율과 후보의 유일성을 확인한다. 큰 공분산은 이동 허가를 막지만 유효한 scan과
TF를 사용한 복구 탐색까지 막지는 않는다. 정지 중 미확인 상태나 복구 검증 중에는
0.5초 간격으로 AMCL의 no-motion update를 요청해 새 관측을 확인한다.
30cm 또는 0.2rad 이상 다른 후보와 점수 차이가 2.5cm 미만이면 모호하다고 거부한다.
AMCL이 새 pose에서 1.5초간 정상 정합을 유지해야 owner가 성공을 반환하며,
Supervisor도 다시 신선한 health를 확인한다. 총 2회까지만 시도하고 실패하면 새
자동 주행을 계속 막는다. 이전 목표를 자동으로 재개하지 않는다.

수동 initialpose, 지도 교체, base 움직임, 입력 중단, 취소는 진행 중인 후보 적용을
중단한다. 큰 위치 오차나 반복 구조 때문에 후보가 모호하면 수동 초기 위치 설정이
필요하다. `localization_evidence_directory`에 회차별 지도·scan·seed와 판정 근거를
저장하며 물리 E-stop 또는 전신 보호 증거를 생성하지 않는다.

## 횡이동과 대각이동의 고정 비용

Phone Omni의 `DirectionalMotionCritic`은 body frame의 이동 각도 θ와 이동 거리로 비용을 계산한다.
이동 1m당 비용은 `1 × sin²(θ) + 20 × sin²(2θ)`이고, 각 예측점에서 속력과
`model_dt`를 곱해 적분한다. `lateral_weight=1.0`, `diagonal_weight=20.0`의 단위는
cost/m이며 일반 plugin 기본값은 각각 0이다. 두 값은 configure 시 읽으므로 변경 후
controller를 재구성해야 한다. 이전 harmonic-overlap 계산과 같은 수치로 호환되지 않는다.

순수 전후진은 0, 횡이동은 1, 45도 대각이동은 20.5 cost/m이다. 횡방향 기본 비용이
더해져 실제 최대는 약 45.36도에서 20.5031 cost/m이며 각도에 따라 연속적으로 변한다.
전후와 좌우 부호에 대칭이고 정지 및 제자리 회전에는 비용을 더하지 않는다.
DiffDrive 복구 controller와 기존 goal checker의 XY 도착 허용오차 안에서는 생략한다.
모든 비용은 유한한 선호이므로 횡이동 후보를 제거하거나 회전 후 전진 순서를 강제하지 않는다.
기존 충돌, 도착, 경로 추종 및 회전 비용과 함께 평가한다.

이 비용은 슬립 측정으로 자동 변경하지 않는다. 위치별 위험 지도와 장기 학습은 구현하지 않는다.
기존 `SlipRecovery`의 감속 정지, 정지 확인, 한 번의 종방향 재시도는 별도로 유지한다.
가중치는 초기 시험값이며 실제 바닥에서의 개선은 실물 비교로 확인해야 한다.

## MPPI 샘플 초기화

`NoiseGenerator`는 초기화와 reset이 반환되기 전에 완전한 잡음 묶음을 생성한다.
`regenerate_noises=true`일 때 이후 묶음은 작업 스레드에서 준비한다. reset은 설정,
운동 모델과 잡음 배열을 같은 잠금 안에서 교체하며, DiffDrive로 전환하면 횡방향 잡음은
0으로 지운다. 각 controller는 독립 난수 엔진을 소유한다. 이 변경은 후보가 모두
같아지는 초기 경쟁 상태를 막지만, 비동기 스케줄까지 결정적으로 만들지는 않는다.
충돌 검사, 비용 가중치와 출력 제한은 바꾸지 않는다.

## Omni 주행 연속성

Phone Omni는 `acceleration_limits=true`로 샘플, 가중 평균, 필터 후 출력과 충돌 회피 대체 후보에 같은 제약을 적용한다. 몸체 X/Y/회전 가속도의 절댓값은 `ax_max=0.20m/s²`, `ay_max=0.15m/s²`, `az_max=0.40rad/s²`이며 네 휠의 명령 가속도는 `wheel_acceleration_max=3rad/s²` 이내다. Twist 전체를 같은 비율로 보간하여 혼합 이동의 휠 제한도 함께 지킨다. 이 모드는 대칭 가감속이며 기존 `ax_min`, `ay_min` 값은 사용하지 않는다. 첫 호출 또는 0.2초 넘는 호출 공백에서는 측정 odom 속도에서 시작하고, 연속 호출에서는 직전 출력에서 시작한다. 측정 초기 속도는 footprint 충돌 예측에 유지한다. 일반 MPPI 프로필의 가속도 제한 기본값은 false다.

Omni의 `path_velocity_weight=0`은 직선 구간을 판별해 횡오차를 벌점으로 주던 별도 규칙을 끈다. `control_variation_weight=1`은 연속 명령 변화에 비용을 추가한다. `path_rotate_enabled=false`와 `goal_rotate_enter_radius=goal_rotate_exit_radius=0`으로 경로·도착 회전 단계의 XY 강제 제한과 그 단계에 연결된 거리 속도 제한을 끈다. 샘플과 출력의 휠 속도 및 가속도 제한은 유지한다.

`proximity_translation_max`와 `proximity_yaw_max`는 local costmap의 치명적 또는
미관측 셀과 padded footprint 외접 사각형 사이 거리로 사전 감속을 계산한다.
`proximity_clearance=0.11m`는 wheel+5cm footprint에서 wheel+16cm 주황 영역까지의
거리다. 셀 반대각을 차감하고, 축 및 휠 가속도 중 보수적인 감속도와
`speed_limit_reaction_time=0.3s`를 반영한다. 0.5m/s에서 0.16m/s로 감속하기 위한
계산상 시작 거리는 주황 경계 약 0.98m 전이다. 이는 실측 정지 거리 보장이 아니다.
목표점 접근에도 같은 제동 곡선을 적용한다. 상한이 낮아질 때 기존 속도는 가감속
제약을 따라 줄이고, 실제 주황 영역의 절대 상한과 빨간 영역 정지는 Collision
Monitor가 적용한다. 입력 단절과 갑자기 나타난 장애물의 정지는 평활화하지 않는다.

### 대칭 고정 자세 이동 후보

`config/lattice/omni-r10cm-symmetric-translation.json`은 비교용으로 보존한 선택적 구성이다.
기존 16개 heading과 5cm 격자를 유지하고, 각 heading에서 몸체 기준 전후, 좌우,
네 대각선의 8방향 이동을 제공한다. 축 이동과 대각선 이동의 길이는 다르지만 반대
방향 및 좌우 반사 쌍은 같은 길이다. 고정 자세 이동 128개와 기존 제자리 회전
32개, 곡선 64개로 총 224개다. 일반 navigation의 기본 파일은 변경하지 않는다.

`tools/validation/native_lattice_mppi/symmetric_translation.py SOURCE OUTPUT --audit AUDIT`
명령으로 후보와 회전 대칭 검사를 재생성한다. `SOURCE`에는 balanced 파일을 준다.
기존 곡선과 제자리 회전의 pose와 길이는 보존한다. 45도 배수 heading의 좌우 회전은
대칭이지만, 나머지 heading은 인접 각도 간격이 달라 반사 쌍이 없다.

후진을 직접 포함하므로 `allow_reverse_expansion=true`와 조합하면 중복 확장이
생길 수 있다. 사용 전 확장 설정과 후진 비용, 경로 품질 및 검색 시간을 검증한다.
구조 검사는 `tests/tools/test_symmetric_primitives.py`, 순정 Smac 파일 읽기는
`native_lattice_mppi` CMake 대상 `primitive_probe`로 실행한다. 이 검사는 실물 주행이나
충돌 없는 경로 생성을 보증하지 않는다.
