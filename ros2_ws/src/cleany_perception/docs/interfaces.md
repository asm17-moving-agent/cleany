# 인식 인터페이스

`inspection_node`의 기존 Action과 토픽은 [런타임 API](runtime.md#ros-api)를 따릅니다.

| 인터페이스 | 변경 계약 |
|---|---|
| `perception/check_rgbd` (`std_srvs/srv/Trigger`) | 요청 뒤 수신한 동기 RGB-D의 시각과 기하를 검사. 추론이나 모터 명령 없음 |
| `DetectedObject2D/3D.label` | 정규화한 최종 종류명. Gemini가 원본 검출 종류를 수정할 수 있음 |
| `DetectedObject2D/3D.detector_label` | 원본 검출 클래스. 미제공은 빈 문자열 |
| `DetectedObject2D.median_depth_m` | 유효 깊이의 중앙값, m |
| `DetectedObject2D.estimated_width_m / estimated_height_m` | 지지면의 두 수평 방향 크기 추정, m. 미측정은 0 |
| `DetectedObject2D.sorting_reason` | 기존 수거 소비자가 사용하는 모델 분류 사유 유지 |

신뢰도는 검출기의 값이며 Gemini 종류명과 처리 분류의 확률이 아닙니다.
동의어 설정은 종류명 정규화만 수행하며 수거 허가 정책을 변경하지 않습니다.
새 필드가 포함된 모든 메시지, Action, Service 소비자는 같은 버전으로 재빌드합니다.

기존 `ObserveWristTarget` 요청의 `detector_label`은 손목 영상의 클래스 대응에
사용합니다. 빈 값은 `label`로 대체하고, CHECK에서 최종 종류명이나 원본 클래스가
바뀌면 거절합니다. 손목 상태와 승인 재확인은 아래 별도 인터페이스를 사용합니다.


## 기준 물체 재관찰과 예열 알림

| 방향 | 기본 이름 | 타입과 조건 |
|---|---|---|
| 서비스 제공 | `perception/observe_object_reference` | `cleany_interfaces/srv/ObserveObjectReference`; `enable_reference_observation=true`일 때 PIN/OBSERVE/CLEAR |
| 발행 | `/perception/wrist_tracking_status` | `cleany_interfaces/msg/WristTrackingStatus`, depth 10; 손목 관찰 결과 |
| 서비스 호출 | `/grasp/vlm_warmup` | `std_srvs/srv/SetBool`; `vlm_ik_warmup_service` 기본값 |
| 서비스 호출 | `/grasp/prepare_window` | `std_srvs/srv/SetBool`; `vlm_anygrasp_prepare_service` 기본값 |

기준 참조는 원래 snapshot/object와 분류를 보존하고 새 RGB-D 관측을 사용한다.
인식, 추적은 같은 추론 잠금을 공유한다. TTL 만료나 새 관측 실패를 성공으로 대체하지 않는다.
예열 알림은 `vlm_ik_warmup_enabled=true`에서만 사용하며 서비스 준비를 기다려 인식을 막지 않는다.

## 승인 대상 재확인

`inspection_node`는 `perception/revalidate_target`에
`cleany_interfaces/srv/RevalidateTarget` 서비스를 제공한다. 원본 snapshot/object와
추적 session/ID를 확인하고 원본과 `after_stamp_ns`(ns)보다 새 RGB-D 한 장에서 대상을
측정한다. 원본 캐시, 좌표계나 카메라 보정이 달라지거나 대응 후보가 모호하면 실패한다.

원본 검출 클래스와 bbox, 마스크로 대상을 대응한다. 응답 한 건은 원본 object/track ID,
종류명, 처리 분류와 `sorting_reason`을 유지하고 신뢰도, 위치, 크기, 깊이는 최신 값을 쓴다.
Gemini 재분류, 추적 이력 갱신, 참조 고정이나 모터 동작은 수행하지 않는다.
진단 snapshot은 파지용 캐시에 저장하지 않으며 승인 실행은 원래 snapshot/object를 쓴다.
인식과 같은 busy lock을 사용하며 실패는 BUSY, RGB-D, MASK, DEPTH, TF, REFERENCE로 구분한다.
승인 유효 기간과 위치 변화 허용치는 [운영 제어](../../cleany_dev_monitor/docs/interfaces.md#승인-기반-작업-제안)가 검사한다.
