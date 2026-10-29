# 승인 대상 재확인

`perception_inspector`는 기본 이름 `/perception/revalidate_target`에
`cleany_interfaces/srv/RevalidateTarget` 서비스를 제공합니다. 기존 관측과 복원
인터페이스는 [런타임 API](runtime.md#ros-api)를 따릅니다.

| 구분 | 계약 |
|---|---|
| 입력 | 캐시된 `source_snapshot_id`, `source_object_id`, 동일한 `tracking_session_id`, `track_id`, 촬영 하한 `after_stamp_ns` |
| 최신 관측 | 원본과 요청 하한보다 새로운 RGB-D, 동일한 카메라 보정값과 좌표계, 촬영 시점 TF |
| 대응 | 원본 YOLOE 클래스와 `reference_minimum_detection_iou` 이상으로 겹치는 후보가 정확히 하나이고 유효한 instance mask가 있어야 함 |
| 유지 | 원본 물체 번호, track ID, 종류명과 처리 분류 |
| 재측정 | YOLOE 신뢰도와 bbox, 유효 깊이, 지지면 기준 크기, bbox 중앙 깊이 기준 3D 대표 위치 |
| 출력 | `success`, `error_code`, `message`, 원본 참조, 최신 대상 한 개의 `detections` |

Gemini를 호출하거나 추적 이력, 원본 캐시 TTL, 집기용 PIN reference를 변경하지
않습니다. 응답의 새 snapshot ID는 진단용이며 파지용 캐시에 저장하지 않습니다.
실행은 원본 승인 snapshot/object를 사용합니다. 소실, 클래스 변경, 중복 후보,
깊이, TF, 보정 변경 또는 만료는 실패입니다. 다른 관측과 동시에 실행하지 않으며
인식기 사용 중에는 `ERROR_BUSY`를 반환합니다.

서비스 성공은 최신 대응과 측정의 성공입니다. 호출자는 승인 유효 기간, 설정,
이동 허용치, bbox 겹침, 신뢰도, 크기와 행동 정책을 추가로 검사해야 합니다.
분류가 정확하거나 물체 교체가 불가능하다는 보장은 아닙니다.
