# 실제 파지 작업 중 IO 통신 계측 (Manipulation IO Probe)

실제 로봇이 물체를 파지하고 수거하는 과정에서 각 ROS 2 토픽의 메시지 수신 주기(Hz), 패킷 간격, 센서 지연(Latency), CPU/GPU 부하를 실시간으로 기록하고 분석하는 도구입니다.

> [!NOTE]
> - 본 도구는 실행 중인 토픽을 수동적으로 구독(Passive Subscribe)하기만 하며, 로봇 구동, 토크 변경, 파지 Goal 전송 등의 명령을 일체 보내지 않습니다.
> - 계측기 프로세스를 강제 종료하더라도 로봇의 제어 루프나 물리 동작에는 아무런 영향을 주지 않습니다.

## 1. 계측 대상 스트림 및 준비

- **관측 스트림 (총 10개 채널)**:
  - 모터 상태: 14관절 통합 위치(`sensor_msgs/JointState`) 및 원시 텔레메트리
  - 헤드 카메라: 컬러 영상(`sensor_msgs/Image`), 정렬된 깊이 영상(`CameraInfo`)
  - 양손목 카메라: 좌/우 RGB 영상 및 각각의 `CameraInfo`
- **오버헤드 최소화 설계**:
  - 모든 영상 스트림은 Best Effort / depth 5로 구독하며, 통신 패킷의 Header만 읽고 **영상 픽셀 데이터는 디코딩하거나 저장하지 않습니다**. (디스크 I/O 부하 방지)
  - 로봇의 1단계~9단계 조작 상태(`request.json`, BT 이벤트) 및 손목 카메라 On/Off 상태를 동기화하여 기록합니다.

## 2. 계측기 사전 검증 (합성 테스트)

```bash
# 1. 단위 테스트
make test-manipulation-io

# 2. 패키지 빌드
make build-manipulation-io

# 3. 가상 합성 환경에서 계측기 스모크 테스트 (약 13초, 모터 구동 없음)
make test-manipulation-io-runtime IO_PROBE_OUTPUT="$PWD/benchmark-results/io-probe-synthetic-01"
```

## 3. 실물 측정 절차

1. **터미널 A (계측기 실행)**:
   ```bash
   # 신규 출력 디렉터리 지정 후 백그라운드 계측 시작 (최대 900초, 작업 완료 시 Ctrl-C 종료)
   make record-manipulation-io \
     IO_PROBE_OUTPUT="$PWD/benchmark-results/grasp-io-trial-01" IO_PROBE_SECONDS=900
   ```
2. **터미널 B (실물 수거 실행)**:
   약 30초의 대기 안정 구간을 확보한 후, `make manipulation-run` 또는 웹 대시보드에서 수거 작업을 1회 실행합니다.
3. **기록 마감 및 리포트 생성**:
   로봇의 복귀가 완료되고 약 30초 후 터미널 A에서 `Ctrl-C`로 계측을 종료하고 리포트를 생성합니다.
   ```bash
   make report-manipulation-io IO_PROBE_OUTPUT="$PWD/benchmark-results/grasp-io-trial-01"
   ```

## 4. 생성되는 분석 보고서 및 그래프

| 산출물 파일 | 내용 및 분석 포인트 |
|---|---|
| `events.jsonl` | 메시지별 수신 타임스탬프, 수신 간격, 패킷 지연 시간, 단계별 리소스 원본 |
| `manifest.json` | 측정 환경, ROS 파라미터, 소켓 버퍼 크기, 실행 완료 여부 |
| `report/stability-timeseries.pdf` | 시간 흐름에 따른 토픽별 Hz, 메시지 간격, 지연 시간, CPU/GPU 사용량 시계열 그래프 |
| `report/message-arrivals.csv` | 보정되지 않은 순수 수신 이벤트 로그 원본 |
| `report/windows.csv` | 1초 슬라이딩 윈도우(100ms 간격) 기준의 평균 Hz 및 비활성(OFF) 구간 |
| `report/phase-metrics.csv` | 조작 단계별(접근, 파지, 들어올리기 등) 최저 Hz 및 최대 지연 시간 |
| `report/summary.json` | 데이터 무결성 요약 (패킷 누락, 지연 이상치, 최종 결과) |

- **그래프 해석 주의점**:
  - 1초 이동 윈도우는 순간적인 패킷 튐을 평활화하므로, 상세 분석 시에는 `message-arrivals.csv`의 개별 간격과 원본 타임스탬프 지연을 함께 검토합니다.
  - 손목 카메라가 OFF 상태인 구간의 긴 간격은 정상적인 비활성으로 취급됩니다.

## 5. 판정 기준 및 경계

도구의 자동 진단 상태는 `SYNTHETIC_TEST`, `INCOMPLETE_RECORDING`, `NO_EXECUTION_RECORDED`, `NO_TERMINAL_RESULT`, `REVIEW_REQUIRED` 중 하나로 출력됩니다.
자동으로 실물 통과(`PASS`)를 판정하지 않으며, 특정 동작 단계에서 패킷 지연 증가나 통신 단절이 로봇의 실제 파지 실패와 연관되어 있는지 엔지니어가 직접 검토해야 합니다.
