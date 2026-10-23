# cleany_moveit_capabilities (MoveIt 고속 배치 상태 검사)

MoveGroup 내부에서 전체 장면에 대한 복수 로봇 관절 상태의 충돌 및 유효성을 일괄 검사하는 배치 플러그인(`cleany/StateValidityBatch`)입니다.
`cleany_moveit_config`의 실물 및 Mock 런타임에 기본 로드되며, IPC(프로세스 간 통신) 오버헤드를 획기적으로 줄여 파지 후보 필터링 속도를 높입니다.

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
