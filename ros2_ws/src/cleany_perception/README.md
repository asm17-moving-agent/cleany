# cleany_perception

## 상태

현재는 설계 경계를 드러내는 scaffold다. ROS 2 package manifest와 node 구현은 아직 없다.

## 역할

카메라, RGB-D, LiDAR 입력에서 객체·공간 상태 후보를 만들고, 객체 후보, confidence,
bbox, depth, 3D pose 후보를 제공한다. 최종 행동 결정은 하지 않는다.

## 제공 계약

입력 토픽, 출력 메시지, confidence 기준, 좌표계는 구현 시 `cleany_interfaces` 및
설정 파일로 명시한다. 낮은 confidence나 안전상 불확실한 결과는 보수적으로 전달한다.

## 설정 및 검증

센서, frame, confidence 임계값은 코드에 고정하지 않고 설정 또는 ROS parameter로
관리한다. 구현 후에는 입력 재생·출력 계약·낮은 confidence 처리를 검증한다.

## 관련 KB

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
