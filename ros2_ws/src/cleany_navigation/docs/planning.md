# 경로 계획과 주행 제어

전역 planner는 지나갈 경로를 찾고, MPPI는 그 경로를 따라갈 속도를 선택한다. 실제 출력은 [안전 경로](safety.md)의 제한을 추가로 받는다. 실행 방법은 [주행 가이드](../../../../docs/how-to/phone-navigation.md)를 따른다.

## 프로필 선택

| 실행 | 전역 planner / primitive | 제어 모델 | 이동 방식 |
|---|---|---|---|
| 일반 AMCL / Omni | Smac Lattice / `omni-r10cm.json` | MPPI Omni | 전후, 횡이동, 회전 |
| Phone 기본 `longitudinal` | StraightFirst → 필요 시 Lattice / 직진과 제자리 회전 | MPPI DiffDrive | 횡이동 없이 직진과 회전 |
| Phone `omni` | Smac Lattice / `omni-r10cm-balanced.json` | MPPI Omni | 방향별 비용으로 전후, 횡이동, 회전 선택 |
| 명시적 `axis_motion:=true` | 축 전용 기존 프로필 | DWB | 축 전용 이동 |

`phone_navigation.launch.py`에는 `robot_params_file`과 기존 안전 센서 입력이 필요하다. Phone 프로필은 일반 navigation 기본값을 바꾸지 않는다.

```mermaid
flowchart TD
    Goal["목표 자세"] --> Mode{"Phone 프로필"}
    Mode -->|omni| Smac["순정 Smac Lattice"]
    Mode -->|longitudinal| Direct{"전체 footprint로 직선 통과 가능?"}
    Direct -->|예| Straight["직선 경로"]
    Direct -->|아니오| Lattice["직진 / 회전 Lattice 탐색"]
    Lattice --> Shortcut["앞쪽 1 m 연결 검사 / 실패하면 원래 경로"]
    Smac --> Omni["MPPI Omni / 이동 방향을 비용으로 선택"]
    Straight --> Diff["MPPI DiffDrive / 경로의 제자리 회전 반영"]
    Shortcut --> Diff
    Omni --> Safety["속도 제한과 충돌 감시"]
    Diff --> Safety
```

## 휴대폰 시연 프로필

### 클릭한 목표와 실제 도착 판정

```mermaid
flowchart LR
    Click["클릭한 좌표와 yaw"] --> Grid["5 cm 셀 / 16개 방향으로 양자화"]
    Grid --> Preview["미리보기 경로의 끝점 / 빨간 목표와 수치 표시"]
    Preview -->|사용자가 이동 선택| Navigate["표시된 자세로 주행"]
    Navigate --> Check["도착 검사 / 3 cm와 0.1 rad"]
```

`tolerance=0`은 다른 목표 셀로 대체하지 않는다는 뜻이다. 연속 좌표의 정확한 보존을 뜻하지 않는다. Controller는 반환된 경로 끝점, 조종기는 표시된 목표를 기준으로 위 허용오차를 검사한다.

### Omni 전역 계획

Phone Omni에는 직선 우선 연결, 고정 방향 XY 탐색, 별도 몸체 방향 탐색이나 거리 기준 방향 선택이 없다.

| 설정 | 값 | 역할 |
|---|---|---|
| `rotation_penalty` | 5.0 | 제자리 회전 비용 |
| `reverse_penalty` | 1.2 | 후진 확장 비용 |
| `non_straight_penalty` / `change_penalty` | 1.2 / 0.05 | 곡선 / 회전 방향 전환 비용 |
| `cost_penalty` | 2.0 | 장애물 비용장 반영 |
| `max_planning_time` | 2.0 s | 검색 시간 예산 |
| `tolerance` | 0.0 m | 다른 목표 셀로 대체하지 않음 |
| `smooth_path` / analytic expansion 길이 | false / 0 m | Primitive 경로 유지 |

### Omni 지역 제어

[nav2_phone_omni.yaml](../config/nav2_phone_omni.yaml)이 Phone 설정을 관리한다. MPPI는 20 Hz로 동작하며 **60 × 0.05 s 예측 구간에 1,000개 후보**를 평가한다.

| 샘플 설정 | 값 |
|---|---|
| 운동 모델 | `Omni` |
| X / Y 속도 표준편차 | 각각 0.1 m/s |
| 회전 속도 표준편차 | 0.2 rad/s |
| Temperature | 0.1 |

| 비용 항목 | 가중치 | 적용 기준 |
|---|---|---|
| PathAlign | 5 | `use_path_orientations=false`: 경로 위치만 평가 |
| Goal / PathFollow | 40 / 30 | 전환 거리 0.5 m |
| GoalAngle | 12 | 목표 0.15 m 안에서 최종 방향 평가 |
| Twirling | 1 | 불필요한 회전 억제 |
| Cost | 설정 파일 참조 | 목표 0.3 m 안에서는 일반 반발 비용 생략, footprint 충돌 검사는 유지 |
| PathAngle | 비활성 | 경로 방향 강제 없음 |

몸체 방향은 [방향별 이동 비용](#횡이동과-대각이동의-고정-비용), 장애물과 회전 비용을 함께 평가해 결정한다. 특정 거리에서 회전→전진→회전을 강제하지 않는다.

### Longitudinal의 경로 연결과 회전

| 상황 | 처리 | 실패 또는 해제 조건 |
|---|---|---|
| 직선 연결 가능 | StraightFirstPlanner가 전체 footprint로 검사 | 불가능하면 Lattice 탐색 |
| Lattice 경로 앞쪽 1 m | 먼 연결 가능 경로점까지 직진과 필요한 제자리 회전으로 연결 | 연속 footprint sweep 또는 최신 costmap 검사 실패 시 원래 경로 |
| 선두 경로점 0.075 m 이내, 같은 XY에서 yaw 변화 | 회전 구간 마지막 yaw를 방향 목표로 선택 | 일반 이동 구간은 기존 경로 방위각 평가 |
| 경로 방향 오차 > 0.1 rad | 후보와 출력의 목표 XY 속도 0 | 가속도 제한 사용 시 먼저 감속, 미사용 시 즉시 XY=0 |
| 목표 위치 3 cm 이내 | 최종 GoalAngle 비용 우선 | 최종 회전 XY 제한은 2.5 cm에서 시작, 3 cm 밖에서 해제 |

`prefix_shortcut_distance=1.0`은 남은 경로와 최종 자세를 보존한다. 다른 프로필의 기본값 0은 이 연결 처리를 끈다. 추가 검색 padding은 0이며 footprint의 기존 5 cm 여유를 검사한다.

경로 회전은 `PathAngleCritic.respect_in_place_rotations=true`와 `path_rotate_enabled`로 제어한다. 측정 초기 속도와 감속 이동도 충돌 예측에 포함한다. Omni는 이 강제 회전 단계를 사용하지 않는다.

## Omni 주행 연속성

### 공간과 속도 상한

| 영역 또는 조건 | 병진 합성 속도 상한 | 회전 상한 |
|---|---|---|
| 정상 | 0.5 m/s | 0.7 rad/s |
| 주황 영역: 바퀴 외곽 사방 0.16 m | 0.16 m/s | 0.3 rad/s |
| 목표점 0.2 m 이내 | 목표 상한 0.06 m/s | 목표 상한 0.15 rad/s |
| 위치 정합 degraded | 0.15 m/s | 0.21 rad/s |
| 빨간 영역, 정지 조건 | 0 | 0 |

병진은 `hypot(vx, vy)`다. 더 낮은 제한이 우선하고 작은 요청을 키우지 않는다. Collision Monitor는 경계를 넘는 혼합 Twist 전체를 같은 비율로 줄여 곡률을 유지한다. 휠 최종 상한 10 rad/s 때문에 병진과 회전을 섞으면 더 느려질 수 있다. Longitudinal 속도는 별도 프로필 값을 따른다.

| 공간 기준 | 크기 / 여유 | 적용 한계 |
|---|---|---|
| CAD 바퀴 포함 하부 외곽 | 0.477 × 0.6548 m | 하부 2D 외곽 |
| Costmap footprint | 위 외곽 + `footprint_padding=0.05 m` | 카메라와 팔 높이의 3D 검사 아님 |
| Collision Monitor 정지 사각형 | 0.577 × 0.7548 m | 실측 제동 거리 보증 아님 |

### 가감속과 장애물 접근

```mermaid
flowchart TD
    Limit["장애물 거리 / 목표 거리 / 위치 정합 제한"] --> Predict["MPPI: 미리 감속할 속도 상한 계산"]
    Predict --> Accel["몸체 축과 네 휠의 가감속 제한"]
    Accel --> Monitor["Collision Monitor"]
    Monitor -->|주황 영역| Scale["절대 상한으로 Twist 축소"]
    Monitor -->|빨간 영역 또는 입력 단절| Stop["즉시 0 / 평활화하지 않음"]
```

| 설정 | Phone Omni 값 | 의미 |
|---|---|---|
| `acceleration_limits` | true | 샘플, 가중 평균, 필터 후 출력, 충돌 회피 대체 후보에 동일 적용 |
| `ax_max` / `ay_max` / `az_max` | 0.20 / 0.15 m/s² / 0.40 rad/s² | 몸체 축별 대칭 가감속; `ax_min`, `ay_min` 미사용 |
| `wheel_acceleration_max` | 3 rad/s² | 네 휠 명령의 가감속 제한 |
| `control_variation_weight` | 1 | 연속 명령 변화 비용 |
| `path_velocity_weight` | 0 | 직선 구간 횡오차에 대한 별도 벌점 비활성 |
| `path_rotate_enabled` | false | 경로 회전의 XY 강제 제한 비활성 |
| `goal_rotate_enter_radius` / `goal_rotate_exit_radius` | 0 / 0 | 도착 회전 단계와 연결된 거리 속도 제한 비활성 |

Twist는 전체를 같은 비율로 보간한다. 첫 호출이나 0.2 s를 넘는 호출 공백에서는 측정 odom 속도, 연속 호출에서는 직전 출력을 시작점으로 쓴다. 측정 초기 속도는 충돌 예측에 유지한다. 일반 MPPI 프로필의 가속도 제한 기본값은 false다.

| 사전 감속 계산 | 기준 |
|---|---|
| 장애물 거리 | 치명적 또는 미관측 costmap 셀과 padded footprint 외접 사각형 사이 거리에서 셀 반대각 차감 |
| 주황 영역까지 여유 | `proximity_clearance=0.11 m`: 바퀴+5 cm에서 바퀴+16 cm까지 |
| 감속도와 반응 시간 | 축 / 휠 중 보수적인 감속도, `speed_limit_reaction_time=0.3 s` |
| 계산 예 | 0.5→0.16 m/s 감속은 주황 경계 약 0.98 m 전부터 시작 |
| 목표 접근 | 같은 제동 곡선 적용 |

이 거리는 계산값이며 실측 정지 거리 보장이 아니다. 낮아진 상한으로의 감속은 가감속 제약을 따르되, 실제 영역의 절대 상한과 갑자기 나타난 장애물의 정지는 Monitor가 적용한다.

## 횡이동과 대각이동의 고정 비용

`DirectionalMotionCritic`은 body frame 이동 각도 θ에 대해 아래 비용을 이동 거리로 적분한다. 각 예측점의 거리는 속력 × `model_dt`다.

```text
이동 1 m당 비용 = 1 × sin²(θ) + 20 × sin²(2θ)
```

| 방향 / 동작 | 비용 (cost/m) | 해석 |
|---|---|---|
| 전진 / 후진 | 0 | 방향 비용 없음 |
| 좌 / 우 횡이동 | 1 | 낮은 추가 비용 |
| 45° 대각이동 | 20.5 | 높은 추가 비용 |
| 약 45.36° | 약 20.5031 | 연속 함수의 실제 최대 |
| 정지 / 제자리 회전 | 추가 비용 없음 | 회전 비용은 다른 critic이 평가 |

전후와 좌우 부호에 대칭이다. **선호 비용이므로 횡이동 후보를 제거하거나 회전 후 전진을 강제하지 않는다.** 충돌, 도착, 경로 추종 비용과 함께 평가하며 DiffDrive 복구 controller와 goal checker의 XY 도착 허용오차 안에서는 생략한다.

| 조정 항목 | 조건 |
|---|---|
| `lateral_weight` / `diagonal_weight` | Phone 1.0 / 20.0 cost/m; 일반 plugin 기본값은 둘 다 0 |
| 값 반영 | configure 시 읽음; 변경 후 controller 재구성 필요 |
| 호환성 | 이전 harmonic-overlap 계산과 같은 수치로 호환되지 않음 |
| 검증 범위 | 초기 시험값; 실제 바닥 개선은 실물 비교 필요 |

슬립 측정으로 가중치를 자동 변경하지 않는다. 위치별 위험 지도와 장기 학습은 미구현이다. SlipRecovery의 감속 정지, 정지 확인, 한 번의 종방향 재시도는 별도 동작이다.

## 위치 정합과 복구

위치가 불확실하면 감속하거나 주행 허가를 회수한다. [위치 정합 문서](localization.md)에서 상태 전이, 정지 조건과 정지 상태의 자동 복구 순서를 확인한다.

| Phone 설정 | 유지하는 기능 | 실행하지 않는 기능 |
|---|---|---|
| Omni `slip_recovery:=false` | slip_monitor와 로그, 기본 Nav2 동작, 위치 정합 복구 | SlipRecovery BT, 종방향 재시도 planner, traction_brake |
| 슬립 BT 활성 | 충돌 감시 입력 `motion/traction_cmd_vel` | — |
| 슬립 BT 비활성 | 충돌 감시 입력 `motion/cmd_vel` | — |

## Lattice 파일과 비교 구성

| 파일 | 구성 | 용도 |
|---|---|---|
| `omni-r10cm.json` | 5 cm 격자, 반경 설정 10 cm, heading 16개, primitive 144개 | 일반 navigation 기본 |
| `omni-r10cm-balanced.json` | 원본 좌표 / 방향 유지; 오른쪽 횡이동 16개의 길이 메타데이터만 교정 | Phone Omni 기본 |
| `omni-r10cm-symmetric-translation.json` | 고정 자세 이동 128 + 제자리 회전 32 + 곡선 64 = 224개 | 선택적 비교 구성 |

파일은 [config/lattice](../config/lattice)에 있다. Launch 기본 경로보다 명시적인 `GridBased.lattice_filepath`가 우선한다. 원본은 공식 Nav2 1.1.20 Generator로 만들었으며 반경 설정은 실물 최소 회전 반경 측정값이 아니다.

### 대칭 고정 자세 이동 후보

비교 구성은 16개 heading과 5 cm 격자를 유지하며, heading마다 전후, 좌우와 네 대각선 이동을 제공한다. 반대 방향과 좌우 반사 쌍의 길이는 같지만 축 이동과 대각선 길이는 다르다.

```bash
# SOURCE: balanced 파일, OUTPUT: 비교 파일, AUDIT: 대칭 검사 결과 경로
python3 tools/validation/native_lattice_mppi/symmetric_translation.py SOURCE OUTPUT --audit AUDIT
```

| 확인 항목 | 조건 / 검사 |
|---|---|
| 기존 primitive | 곡선과 제자리 회전의 pose 및 길이 보존 |
| 회전 대칭 | 45° 배수 heading은 좌우 대칭; 나머지는 인접 각도 간격이 달라 반사 쌍 없음 |
| 후진 확장 | 후진 primitive를 직접 포함하므로 `allow_reverse_expansion=true`와 중복 가능 |
| 사용 전 비교 | 확장 설정, 후진 비용, 경로 품질과 검색 시간 |
| 구조 검사 | `tests/tools/test_symmetric_primitives.py` |
| 순정 Smac 파일 읽기 | `native_lattice_mppi` CMake 대상 `primitive_probe` |

이 검사는 실물 주행이나 충돌 없는 경로 생성을 보증하지 않는다.

## MPPI 샘플 초기화

| 시점 / 범위 | 계약 |
|---|---|
| 초기화와 reset 반환 전 | 완전한 잡음 묶음 생성 |
| `regenerate_noises=true` | 이후 묶음을 작업 스레드에서 준비 |
| reset 잠금 | 설정, 운동 모델과 잡음 배열을 함께 교체 |
| DiffDrive로 전환 | 횡방향 잡음 0으로 초기화 |
| Controller별 상태 | 독립 난수 엔진 소유 |

후보가 모두 같아지는 초기 경쟁 상태를 방지하지만 비동기 스케줄의 결정성은 보장하지 않는다. 충돌 검사, 비용 가중치와 출력 제한은 별도 계약이다.

커스텀 controller는 `cleany_navigation::MPPIController`, critic은 `cleany_navigation::mppi::critics`로 등록한다. 소스는 `mppi/src`와 `include/cleany_navigation/mppi`, 원본 설명과 라이선스는 [upstream README](../mppi/README.upstream.md)와 [LICENSE](../mppi/LICENSE.md)에 있다.
