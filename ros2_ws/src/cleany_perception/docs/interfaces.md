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
바뀌면 거절합니다. 새 손목 상태 발행 토픽이나 승인 재검증 서비스는 추가하지 않습니다.
