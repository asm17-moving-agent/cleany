# 실제 파지 작업 중 IO 계측

이 도구는 기존 실물 파이프라인을 관측한다. 로봇 시작, 토크 변경, 파지 Goal,
카메라 활성화와 정지 명령은 보내지 않는다. 기록기 종료도 로봇을 정지시키지 않는다.
기존 실행 경로와 보호 설정은 `cleany_skill_executor/README.md`를 따른다.

## 준비 상태와 시험 범위

- 기본 토픽: 모터 통합 위치와 원시 피드백, 헤드 컬러와 정렬 깊이,
  양손목 RGB 및 각 CameraInfo, 총 10개 스트림.
- 모든 스트림은 Best Effort / depth 5로 관측한다. 원시 CDR에서 영상 Header만
  읽고 픽셀을 디코딩·압축·저장하지 않는다. 추가 구독 자체의 DDS 부하는 남는다.
- 영상의 원시 깊이와 미리보기 토픽은 기본 구독에 추가하지 않는다. 실제 운영에서
  필요 없는 영상 발행·인코딩 부하를 계측 때문에 새로 만들지 않기 위해서다.
- 모터 14개 위치는 하나의 JointState 메시지에 함께 들어온다. 수신 시각은
  개별 모터의 버스 응답 시각과 다르다. 상세 피드백의 원본 `feedback_stamp_ns`를
  별도 확인하며 해당 필드가 없는 헤드 13–14번은 신선도 미측정으로 남는다.
- 실행 이벤트의 execution ID, stage, substage, result를 기록한다. Action 이전
  런타임 준비와 장면 인식은 기존 request.json을 1초마다 관측하고 source history도 보존한다.
- `/manipulation/bt_transitions`와 손목 `/enabled` 상태 변경·heartbeat를 함께 저장한다.
  손목 상태가 3초 넘게 오래되면 UNKNOWN이다. 상태가 확인된 OFF만 정상 미사용으로 구분한다.
- 기본 운영 프로필 `collection_hardware.json`은 현재 손목 관측과 fallback을 끈다.
  그대로 시험하면 손목은 OFF가 예상된다. 손목 사용 시나리오는 기존 팔 선택·활성화
  경로를 사용하는 별도 설정의 시험으로 구분하며, 기록기가 두 손목을 강제로 켜지 않는다.
- 현재 `pick_return_release`와 `temporary_placement_verification=assume_success`는
  수거함 안착을 실제 확인한 조건이 아니다. ASSUMED 결과를 CONFIRMED로 바꾸지 않는다.

## 빌드와 계측기 검증

저장소 루트에서 실행한다. 이미 실행 중인 로봇을 이 명령들이 재시작하지 않는다.
새 `/enabled` 상태는 업데이트된 손목 노드를 다음에 시작할 때 제공된다.

```bash
make test-manipulation-io
make build-manipulation-io
make test-manipulation-io-runtime IO_PROBE_OUTPUT="$PWD/benchmark-results/io-probe-synthetic-01"
```

마지막 명령은 localhost의 별도 ROS domain 181에서 합성 센서와 단계 이벤트를
약 13초 발행한다. 실제 모터, 카메라, 파지 Action은 사용하지 않는다.
의도적인 영상 공백, 오래된 이미지 stamp, 반복된 모터 피드백, 손목 ON/OFF,
단계 전환과 terminal 기록을 확인한다. 결과에 SYNTHETIC_TEST를 명시한다.
이 합성 시험은 발행기와 기록기가 같은 프로세스를 공유하므로 CPU/RSS 표본을
운영 기록기의 단독 오버헤드로 해석하지 않는다.

## 실제 시험 순서

실물 실행은 기존의 승인된 물체·배치와 감독 하에 진행한다. 감독자, 작업 구역과
비상 정지 준비는 KB의 Real Robot 시험 조건을 따른다. 여기서는 새로운 물리
성공 기준이나 보호 한계를 정하지 않는다.

1. 같은 물체·배치·팔·모델·운영 설정을 유지하고 회차마다 새 출력 경로를 정한다.
   첫 기동 회차와 모델 준비가 끝난 후 회차를 구분한다. 정상 경로는 우선 3회씩
   기록하되 이 횟수를 통계적인 안정성 보장으로 해석하지 않는다.
2. 터미널 A에서 아래 기록 명령을 먼저 실행한다. 시작/종료 상태도 수집된다.
3. 준비된 운영 런타임에서 약 30초 대기 구간을 남긴 뒤, 터미널 B에서 기존
   `make manipulation-run` 또는 동일한 웹의 물체 1개 실행을 한 번 수행한다.
   이 명령은 실제 구동 명령이므로 감독자가 실행한다. 기록 명령과 자동 연결하지 않았다.
4. 인식 → 후보·경로 선택 → 접근 → 파지 → 상승 → 복귀/방출 → 종료까지 이어서
   기록한다. 완료 후 약 30초를 남기고 터미널 A의 Ctrl-C로 기록만 마친다.
   기본 900초 제한에 걸리면 terminal 없는 부분 기록으로 남으므로 필요 시 길이를 늘린다.
5. 실물 관찰로 파지 유지, 낙하·전도, 방출과 실제 물체 상태를 회차 메모에 남긴다.
   실행 결과의 SUCCESS나 메시지 수신 정상만으로 이 항목을 대체하지 않는다.

```bash
# 터미널 A: 새 경로여야 한다. 900초 이내 Ctrl-C로 조기 마감 가능.
make record-manipulation-io \
  IO_PROBE_OUTPUT="$PWD/benchmark-results/grasp-io-trial-01" IO_PROBE_SECONDS=900

# 기록 종료 후: PDF, PNG, CSV와 단계별 수치를 생성한다.
make report-manipulation-io IO_PROBE_OUTPUT="$PWD/benchmark-results/grasp-io-trial-01"
```

`CLEANY_COLLECTION_ROS_DOMAIN_ID`로 운영 domain을 맞춘다. 기록 대상 Jetson에서
실행해야 CPU/GPU·호스트 UDP와 로컬 command history를 함께 확인할 수 있다.
직접 ROS 명령의 `--config`, `--request-file`, `--label`, `--seconds`로 설정을
바꿀 수 있다. 기본 설정은 `config/manipulation_io_probe.json`이다.
실행 시 운영 설정 사본과 SHA-256, ROS 환경, 수신 버퍼 상한을 manifest에 남긴다.

## 기록과 그래프 읽기

| 파일 | 내용 |
|---|---|
| `events.jsonl` | 메시지마다 수신 시각·간격·stamp 지연, 100ms 창, 단계, 상태와 자원 |
| `manifest.json` | 설정·환경·수집 종료 여부·writer 유실·최종 실행 기록 |
| `latest.json` | 가장 최근 호스트/기록기 자원 표본, 파일 기록 진행 확인용 |
| `report/stability-timeseries.pdf` | 시간별 Hz·메시지 간격·단계·신선도·CPU/GPU·UDP 그래프 |
| `report/message-arrivals.csv` | 수신 이벤트 원본, 평활화하지 않은 간격과 원본 stamp |
| `report/windows.csv` | 기본 1초 창을 100ms마다 계산한 Hz와 OFF/UNKNOWN 상태 |
| `report/phase-metrics.csv` | execution ID와 단계별 최저 창 Hz·최대 활성 간격 |
| `report/phase-events.json` | 준비 단계 history·실행기 결과·BT·손목 상태 원본 |
| `report/summary.json` | 기록 완결성, 누락·상태 불명·지연·종료 결과 |

1초 창은 전체 평균과 다르지만 1초 안의 짧은 정체를 가릴 수 있다. 오른쪽의
메시지별 간격과 원본 stamp 지연을 함께 본다. OFF→ON 사이의 긴 간격은 원본
CSV에 유지하고 활성 간격 그래프에서만 제외한다. 첫 1초 미완성 창과 켜진 뒤
2초의 준비 구간도 구분한다. 0Hz 구간과 마지막 수신 이후 공백을 숨기지 않는다.

모터 상세 필드는 기본 5Hz로 저장하고, 각 50Hz 메시지의 최대 원본 피드백 나이,
같은 피드백 반복 여부, 오류와 복구 카운터는 메시지마다 남긴다. CPU/RAM/UDP는
1Hz이며 `tegrastats`가 사용 가능하면 GPU와 전력·온도 원문도 1Hz로 보존한다.
GPU가 수집되지 않으면 미측정이다. 센서가 timestamp를 붙인 방식과 시계 차이가
지연에 반영되며 이 값을 물리적 노출부터의 전체 지연으로 해석하지 않는다.

기록기 자체 CPU·RSS, timer 지연과 writer queue도 함께 확인한다. queue 초과,
저장 용량 제한 또는 디스크 오류는 불완전 기록으로 분류한다. 최대 저장량의 기본은
512MiB이며 영상 픽셀은 저장하지 않는다. 계측기가 작업에 미치는 영향을 분리하려면
동일 조건의 기록기 OFF/ON 회차를 추가 비교해야 한다.

## 판정 경계

자동 출력은 SYNTHETIC_TEST, INCOMPLETE_RECORDING, NO_EXECUTION_RECORDED,
NO_TERMINAL_RESULT 또는 REVIEW_REQUIRED다. 자동으로 실물 안정성 PASS를 내리지 않는다.
소스 주기 대비 긴 간격, 활성 중 0Hz, stale 피드백, UDP 증가, 상태 불명과 실제
작업 실패가 같은 단계에서 재현되는지 검토한다. 허용 지연·성공률은 아직 확정된
제품 요구사항으로 취급하지 않으며 현재 제어기의 보호 조건도 변경하지 않는다.

운영 구간을 기록하지 않은 준비 시험과 실제 물체를 다룬 회차를 반드시 구분한다.
