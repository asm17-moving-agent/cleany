# cleany_logger

## 상태

현재는 설계 경계를 드러내는 scaffold다. ROS 2 package manifest와 logger 구현은 아직 없다.

## 역할

Cleany event log와 failure code 기록을 담당한다. 실패 원인은 `DETECTION_FAIL`,
`DEPTH_FAIL`, `LOW_CONFIDENCE`, `OUT_OF_WORKSPACE`, `GRASP_FAIL`, `PLACE_FAIL`,
`NAVIGATION_FAIL`, `COLLISION_RISK`, `HARDWARE_ERROR`, `TIMEOUT`, `UNKNOWN_OBJECT`,
`USER_INTERVENTION_REQUIRED` 같은 구조화된 코드로 남기는 방향을 검토한다.

## 제공 계약

로그 스키마, 보존 위치, 외부 전송 여부와 failure code의 최종 목록은 구현 및 사람 검토와
함께 확정한다. 이 패키지는 상태 전이나 실패 복구 결정을 내리지 않는다.

## 설정 및 검증

로그 레벨, 저장 위치, 개인정보·보존 정책은 설정으로 분리한다. 구현 후에는 event와
failure code가 재현 가능한 형식으로 기록되는지 검증한다.

## 관련 KB

- [Data and Evaluation](../../../docs/cleany-docs/20_TECHNICAL/07%20-%20Data%20and%20Evaluation.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
