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

## 수집·표시 한도

| 항목 | 한도 |
|---|---|
| ROS discovery / UI 갱신 | 1 Hz / 최대 10 Hz |
| Action / 사용자 선택 토픽 / 브라우저 | 32 / 16 / 8 |
| 미리보기 전체 / 배열 / 깊이 / 문자열 | 2048 / 256 / 10 / 4096자 |
| 지도 기본 / 설정 상한 | 524288 / 1048576셀 |
| 경로 / 이벤트 ring / 브라우저 로그 | 4096점 / 2048건 / 500건 |
| 느린 브라우저 전송 timeout | 2초 |

- Metadata 전체 discovery; 메시지는 기본·action·선택 토픽만 구독
- 미구독: unmeasured. 마지막 선택자 해제 시 구독 정리
- 수신 주기: 발행 주기 보장 아님
- QoS: publisher endpoint 기준; 혼합 publisher는 호환 가능한 약한 설정
- 오류 구분: type import·QoS·publisher 없음·수신 없음·stale
- 미리보기: 한도 내 변환, truncated 표시. 전체 cloud/rosbag 대체 아님
- 초과 지도: 거절 사유 표시. 초과 경로: 잘림 표시
- 2D: 메시지 시각 TF → map_frame, 회전 origin 반영. TF 없으면 숨김
- Robot pose: 최신 TF, ±2초 밖 stamp는 stale. 정적 지도·마지막 경로 유지
- Telemetry: 종류/토픽별 최신값. 밀려난 이벤트: gap 표시
- 재접속: 초기 snapshot 제공. Runtime/Monitor boot ID 별도

## 기록·판정

- 관측 전용. Runtime의 안전·완료 판정 대체 금지
- 단절·재시작·sequence gap 표시. IDLE/성공/안전으로 추정 금지
- 저장 시각: ROS stamp + 수신 wall/monotonic. 재생 축: 수신 wall
- ROS clock 역행: 이벤트 기록. Clock 정체·수신 지연 표시
- Host wall 역행: 정밀 재생 미지원
- 선택 기록: 전체 rosbag 대체 아님. Camera는 별도 경로
- HTTP 경로·허용 항목: server 구현·monitor.yaml 기준
- 기록: 외부 cleany-data. web/dist·ROS install: 생성물
- 안내: [실행](../../../../docs/RUNNING.md), [환경](../../../../docs/DEVELOPMENT_SETUP.md)
