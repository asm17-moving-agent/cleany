# Monitor 관측 데이터 계약


```text
MissionRuntime ─ debug_snapshot (2 Hz), runtime_events ─┐
ROS graph / selected topics / rosout / TF ──────────────┤
                                                      ▼
                 cleany_dev_monitor (rclpy, 관측만)
                         bounded Hub
                              │ 별도 worker thread
                        aiohttp + SQLite
                              │ HTTP / WebSocket
                        React + Vite + TypeScript + shadcn/ui
                         React Flow / Canvas
```

- ROS discovery 1 Hz, 선택 메시지/위치/웹 갱신 최대 10 Hz. 수신 주기는 발행 주기를
  보장하지 않으며 미구독 토픽은 `unmeasured`로 표시한다.
- 모든 토픽의 **메타데이터**를 찾지만 모든 메시지를 구독하지 않는다. 기본 관측 토픽,
  최대 32개 action status/feedback 토픽, 사용자가 선택한 최대 16개 토픽만 구독한다.
  최대 8개 브라우저가 구독을 공유한다. 마지막 선택자가 해제/종료하면 구독을 정리한다.
- QoS는 발견된 publisher endpoint에 맞춰 reliable/best-effort와
  transient-local/volatile을 선택한다. 혼합 publisher에는 호환 가능한 가장 약한
  설정을 사용한다. 타입 import 실패, QoS 오류, publisher 없음, 수신 없음, 오래된 수신을 구별한다.
- 메시지를 JSON으로 전부 펼친 뒤 자르지 않는다. 미리보기는 총 2,048개 항목,
  배열당 256개 항목, 깊이 10, 문자열당 4,096자 한도로 변환한다. `truncated`를 표시한다.
  PointCloud 원본 보기와 rosbag 전체 데이터 저장을 대체하지 않는다. 카메라 영상은 별도 경로로 제공한다.
- 지도는 기본 최대 524,288셀, 경로는 최대 4,096점이다. 초과 지도는 거절 사유를
  표시한다. 경로는 잘린 여부를 데이터에 남긴다. `max_grid_cells` 상한은 1,048,576이다.
- 2D 레이어는 메시지 시각의 TF를 이용해 `map_frame`으로 변환한다. 회전된 지도
  origin도 반영한다. TF가 없으면 레이어를 표시하지 않는다. 로봇 위치는 최신 TF를
  사용하고 ±2초 밖의 stamp는 오래된 값으로 처리한다. 정적 지도/마지막 경로는 유지된다.
- 최근 telemetry는 종류/토픽별 최신값으로 합치고, 이벤트/로그/결과는 별도 2,048개 ring에
  보존한다. 오래된 이벤트가 밀려나면 gap을 표시한다. 브라우저 로그는 최근 500개다.
  느린 브라우저의 전송은 2초 후 종료하고 재접속 시 초기 snapshot을 다시 제공한다.
- Runtime boot ID/sequence와 Monitor boot ID를 분리한다. 재접속, Runtime 재시작,
  이벤트 sequence gap을 표시한다. 데이터가 끊겨도 IDLE/성공/주행 안전으로 바꾸지 않는다.
- ROS stamp와 수신 wall/monotonic 시각을 함께 저장한다. 재생 축은 수신 wall 시각이다.
  `/clock` 역행은 이벤트로 남기며, 같은 clock 값 유지/수신 지연을 화면에 표시한다.
  호스트 wall clock 자체가 역행하는 환경의 정밀 재생은 지원하지 않는다.


## 기록과 영상

관제보드는 관측 전용이며 Runtime의 안전·완료 판정을 대체하지 않는다.
SQLite 기록과 재생은 수신 wall 시각을 사용한다. ROS clock jump와 이벤트 gap을 보존한다.
선택 항목의 기록은 rosbag 전체 원본 저장을 대체하지 않는다.
카메라 영상·메타데이터의 정확한 HTTP 경로와 허용 항목은 server 구현과 monitor.yaml을 따른다.
기록 데이터는 external cleany-data 아래에 두며 web/dist와 ros2 install은 생성물이다.
실행과 웹 빌드는 [RUNNING](../../../../docs/RUNNING.md), [환경 준비](../../../../docs/DEVELOPMENT_SETUP.md)를 따른다.
