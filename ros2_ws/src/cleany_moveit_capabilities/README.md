# cleany_moveit_capabilities (MoveIt 고속 배치 상태 검사)

MoveGroup 내부에서 전체 장면에 대한 복수 로봇 관절 상태의 충돌 및 유효성을 일괄 검사하는 배치 플러그인(`cleany/StateValidityBatch`)입니다.
`cleany_moveit_config`의 실물 및 Mock 런타임에 기본 로드되며, IPC(프로세스 간 통신) 오버헤드를 획기적으로 줄여 파지 후보 필터링 속도를 높입니다.

`cleany/SolidFCL` 충돌 플러그인은 최적화 메시의 닫힌 볼록 조각을 FCL Convex로
읽습니다. 삼각형 표면을 가로지르지 않고 조각 내부에 완전히 포함된 장애물도
검출하며, 조각 사이의 틈과 링크별 좌표계는 유지합니다. 오목하거나 열린 조각은
초기화를 거부합니다. world, ACM, attached object와 거리 쿼리는 기존 FCL 처리를
사용합니다. 부착 메시와 world 메시 자체를 볼록 형상으로 바꾸지는 않습니다.

자기 충돌 검사는 매 요청의 자세로 링크별 바깥 경계 상자를 다시 만들고,
서로 떨어져 있거나 현재 ACM에서 항상 허용한 링크 쌍을 먼저 제외합니다.
남은 부품 쌍은 기존 FCL callback으로 검사하므로 조건부 허용과 attached
touch link도 적용됩니다. 이전 자세의 충돌 결과는 재사용하지 않습니다.
거리 요청은 기존 FCL 탐색을 사용하며, 비어 있는 world의 일반 충돌 요청은
로봇 검색 구조를 만들지 않습니다.

실물, Mock와 시뮬레이션의 MoveIt launch가 이 detector를 선택합니다. 배치
capability 초기화 시 요청한 detector와 실제 detector가 다르면 node를 종료합니다.
원본 CAD를 비교할 때는 `collision_detector:=FCL`을 명시해야 합니다.

OMPL의 `cleany/RobustTimeParameterization`은 양팔에 같은 TOTG 시간 배정을
적용합니다. 거의 되돌아가는 꺾임에서 고정 수치 오차 기준이 짧은 곡선
구간을 가로지르던 문제를 구간 길이에 맞는 기준으로 수정했습니다. 적분
간격도 곡률, 감속과 다음 구간 경계를 반영해 짧은 꺾임을 건너뛰지 않습니다. 끝
시각이 샘플 주기 경계에 걸릴 때 거의 같은 시각이 두 번 생기는 문제도
균등한 시간 표본 배치로 수정했습니다. 별도의 대체 시간 배정기는 없습니다.
구현 출처와 수정 범위는 [TOTG 출처](src/totg/README.md)를 참고합니다.

직선 내부의 중복 점은 제거된 모든 점을 최종 직선과 다시 대조해 정리합니다.
길이가 짧다는 이유로 구간을 버리지 않으며, 정확한 방향 반전은 유한한
가속도에 필요한 속도 0 경계 조건으로 같은 TOTG에서 계산합니다.
꺾임의 중앙과 정지 경계를 기준으로 표본을 배치해 서로 거의 겹치는
시간 표본이 생기는 것을 막습니다.

반환된 위치, 속도와 가속도의 5차 보간 전체를 Bernstein 계수로 감쌉니다.
관절 위치 한계와 정리된 경로의 `path_tolerance` 범위에 들어가도록 필요한
구간만 세분합니다. 내부 곡선 생성에는 허용오차의 절반을 사용합니다.
ROS 시간의 나노초 반올림도 보간 형상 및 속도와 가속도 상한에 반영합니다.
미분 상한에 따른 전체 시간 배율은 보간 형상과 관절별 동기화를 유지합니다.
이 관절 공간 오차는 실제 공간의 충돌 여유를 뜻하지 않습니다. 실물
실행기는 기존대로 명령 경로를 별도 시간 배정하고 충돌 검사합니다.

관절별 제한과 요청의 velocity/acceleration scaling을 적용합니다. 제한이
없는 관절의 기본값은 `ompl.retiming_default_velocity_limit` 및
`ompl.retiming_default_acceleration_limit`이며 각각 1.0입니다. 이는 기존
MoveIt의 계획용 기본값이며 실물 모터 제한의 승인 근거가 아닙니다.
`ompl.retiming_collinear_tolerance` 기본값은 1e-12입니다. 작은 움직임을
임의로 없애지 않도록 `ompl.min_angle_change` 기본값은 0입니다. 샘플로
표현할 수 있는 정지와 극소 왕복도 계산합니다. 유효한 시간 궤적을 만들지 못하는 입력은
`INVALID_MOTION_PLAN`과 빈 궤적으로 거부하며 원래 입력은 보존합니다.

## 1. 인터페이스 규격

- **서비스 명칭**: `/check_state_validity_batch`
- **서비스 타입**: `cleany_interfaces/srv/CheckStateValidityBatch`

| 파라미터 / 반환값 | 타입 | 설명 |
|---|---|---|
| **요청 (`request`)** | `RobotState[]` (최대 64개) | 검사할 관절 상태 배열 (기본 청크 크기: 32개) |
| 〃 | `group_name`, `constraints` | 계획 그룹명 및 공통 자세 제약 조건 |
| **응답 (`response`)** | `bool success` | 연산 성공 여부 (계산 실패 시 false) |
| 〃 | `bool[] valid` | 각 상태별 충돌 및 제약 유효성 결과 (최초 실패 상태까지 검사) |
| 〃 | `ContactInformation[] contacts` | 충돌 발생 시 접촉한 링크 및 장애물 정보 |

## 2. 동작 특징 및 일관성 보장

1. **단일 씬 잠금**:
   - 한 번의 서비스 요청 동안 MoveIt Planning Scene과 OctoMap의 **읽기 잠금(Read Lock)**을 연속 유지하며 순서대로 상태를 판정합니다.
   - 요청과 요청 사이에는 씬 갱신(OctoMap 업데이트) 및 취소/안전 감시 루프가 정상 동작합니다.
2. **엄격한 충돌 판정**:
   - 현재 활성화된 ACM(Allowed Collision Matrix), 부착된 물체(Attached Object), 옥토맵 포인트 클라우드가 모두 동일하게 적용됩니다.
   - 단일 상태 검사(`/check_state_validity`)와 100% 동일한 유효성 기준을 보장합니다.
3. **안전 Fallback**:
   - 배치 기능이 없는 다른 MoveIt 환경에서는 클라이언트가 자동으로 기존 단일 상태 검사 루프로 Fallback합니다.

## 3. 빌드 및 테스트

```bash
# C++ 플러그인 빌드
make build-manipulation-speed

# 단위 및 런타임 검증
make test-manipulation-speed
```

- **검증 내용**: 가상 장애물 및 OctoMap 환경에서 샘플 누락 여부, 최초 충돌 검출 위치, ACM 허용/거부 전환, 옥토맵-링크 접촉 판정의 정확성을 검증합니다.
