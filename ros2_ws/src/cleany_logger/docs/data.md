# Logger 기록 데이터 계약

`logger_node`가 ROS 구독, 가공 관측 생산과 저장을 소유한다. Monitor는 실시간 표시와
ROS 제어 client, `ArchiveReader`를 사용하는 읽기 전용 HTTP adapter다. Logger는
Monitor와 aiohttp, 카메라 preview 디코더, Mission Runtime을 import하지 않는다.
rosbag2 의존성은 첫 원본 쓰기에 불러오며 저장 API와 과거 조회는 ROS 없이 사용할 수 있다.
독립 실행과 상태 전이는 [런타임](runtime.md), 서비스 계약은 [인터페이스](interfaces.md)를 따른다.

## 생산자와 저장 소유권

| API | 입력과 계약 |
|---|---|
| `ObservationBuffer.put` | 가공 관측의 수신 wall 시각을 초 단위 `received_at`으로 전달할 수 있다. 생략하면 현재 wall 시각을 사용한다. |
| `RecordingControls` | 항목, 주기, 출처와 원본 큐를 관리한다. `reserve`로 얻은 ticket을 `offer`에 전달한다. |
| `RawMessage` | 토픽, 메시지 타입, 변경하지 않은 CDR bytes, 수신 wall 초, 선택적 ROS stamp 초와 정수 수신 ROS timestamp ns를 전달한다. |
| `Recorder.append(snapshot)` | `sequence`, `rows`, 선택적 `gap`이 있는 snapshot을 기록한다. 각 row는 `seq`, `kind`, `key`, `data`, `received_at`을 포함한다. |
| `Recorder.append_raw()` | 제어 큐를 비우고 rosbag2에 쓰며 SQLite에는 원본 위치만 남긴다. |
| `Recorder.restore_settings(config)` | 저장된 정책을 현재 출처와 함께 검증해 복원한다. 없거나 읽을 수 없는 설정은 기본값을 유지한다. |
| `Recorder.persist_settings(settings)` | 검증한 정책을 임시 파일, fsync와 rename으로 원자적으로 저장한다. |
| `Recorder.start_session()` | 이전 writer를 종료하고 현재 한도와 같은 controls, bag factory로 새 UUID 세션을 연다. |
| `Recorder.close()` | bag을 종료하고 SQLite를 commit한 뒤 닫는다. |
| `export_sqlite`, `export_bag` | 가공 DB의 일관된 backup과 종료된 native bag ZIP을 만든다. HTTP 응답과 임시 파일 수명은 adapter가 소유한다. |
| `ArchiveReader(directory)` | 경로만 보관하며 디렉터리, 설정 파일과 세션을 생성하지 않는다. |
| `ArchiveReader.sessions`, `replay`, `raw_message`, `navigation_runs` | 세션 metadata, 시각별 가공 관측, CDR 한 건과 제한된 주행 요약을 읽는다. |

생산자는 가공을 마친 JSON 호환 row를 전달한다. `received_at`은 wall 초이며
`received_monotonic`은 선택적 표본화 시계다. `ros_time`, `source_id`, `category`,
`topic_revision`으로 원본 시각, 개별 기록 선택과 출처 세대를 전달할 수 있다.
같은 생산자의 `seq`는 증가해야 하며 출처 변경 시 `topic_revision`을 증가시킨다.

`Recorder`와 SQLite 연결은 호출한 단일 worker가 소유하며 모든 쓰기와 종료를 같은
thread에서 수행한다. ROS callback은 thread-safe한 `RecordingControls`만 사용한다.
새 세션 전에는 `controls.transition()`으로 callback ticket을 봉인하고 이전 snapshot과
원본 큐를 비운다. 이후 새 세션을 열고 생산자가 초기 snapshot을 전달한다. Monitor의
시각별 HTTP 경로와 구독 계약은 [Monitor 계약](../../cleany_dev_monitor/docs/data.md)에 있다.

logger는 작업의 성공이나 실패를 판정하지 않고 생산자가 보낸 이벤트, 결과와 실패 코드를
기록한다. `MissionJournal`의 실행 복구와 상태 소유권은 Mission Runtime에 남아 있다.

## 저장 형식과 시간

ROS stamp와 수신 wall/monotonic 시각을 함께 저장하며 재생 축은 수신 wall이다. ROS clock 역행은 이벤트로 남기고 clock 정체와 수신 지연을 표시한다. Host wall 역행 환경의 정밀 재생은 지원하지 않는다.

저장 경계는 ROS 메시지의 CDR 원문은 rosbag2, logger가 만든 조회와 재생 모델은 SQLite다. 선택하고 표본화한 기록이므로 전체 ROS 그래프의 무손실 녹화를 뜻하지 않는다.

| 데이터 | 저장소 |
|---|---|
| 선택한 센서, 지도, 경로, 위치, Runtime, BT, 안전 진단, 로그와 SLAM 원문 | rosbag2의 표준 `sqlite3` storage |
| TF, 시계, 카메라 CameraInfo와 선택한 로봇 모델 문맥 | 같은 bag에 수신마다 기록 |
| 화면 레이어, 미션, BT, 안전 상태, 로그와 호스트 리소스 | 가공 SQLite의 가공 자료 |
| 원본의 토픽, 타입, 수신 시각과 bag timestamp | SQLite `raw_index`; payload 없음 |

`state_directory` 아래 `<session>.sqlite3`, `bags/<session>/`, `recording-settings.json`을 둔다. 기본 위치는 `~/.local/state/cleany/recordings`이며 실행 시 `-p state_directory:=/absolute/path`로 바꾼다. 기록은 자동 삭제하지 않는다. `web/dist`와 ROS install은 생성물이다.

ROS callback은 최대 8 MiB 메시지를 32 MiB/128건의 큐로 넘기고 logger 전용 writer thread가 bag 쓰기와 종료를 소유한다. 초과, 쓰기 실패와 용량 중단은 유실 건수와 `raw_gap` metadata로 남긴다. rosbag2 runtime이 없거나 쓰기에 실패하면 원본 기록을 중단하며 SQLite payload로 대체하지 않는다. 세션 제한과 `total_limit_bytes`는 SQLite와 bag을 합산한다.

정지 후 녹화를 시작할 때 새 UUID를 만들고, 녹화 중 시작 요청은 같은 세션을 반환한다. 정지하면 SQLite와 bag writer를 닫는다. `bag_state`는 `not_started`, `recording`, `finalized`, `incomplete`를 구분하며 완료된 bag만 ZIP으로 내보낸다. 원본 timestamp는 수신 ROS clock의 nanosecond, SQLite 시간축은 수신 wall 초다. 동일 ROS timestamp의 원본도 수신 순서로 구분한다. 이전 schema 1/2/3 SQLite는 수정하거나 자동 변환하지 않고 계속 읽는다.

원본 구독은 화면과 카메라 preview lease에 독립적이다. CameraInfo는 이미지 토픽의 형제 `camera_info`에서 요청한다. TF와 시계, CameraInfo는 Hz로 누락시키지 않는다. 설정 변경과 새 세션은 transient-local 원본 구독을 다시 생성해 retained 자료를 요청한다.

## 조작 이력

공유 관측기는 조작 execution record의 revision과 서버 wall timestamp로 실행별
단계 구간을 집계한다. Logger와 Monitor가 각각 독립적으로 집계하며 기본 최근
20개 실행, 실행당 64개 구간을 보존한다. 늦은 구독, revision 누락과 시각 역행을
표시하며 수신 시각 이후의 결과를 과거 재생에 섞지 않는다. Logger의 관측 출처를
바꾸면 조작 이력과 revision을 초기화한다. 집계 결과는 해당 조작 출처의 녹화
선택을 따르며 Monitor나 HTTP 모듈을 필요로 하지 않는다.

## 기록 정책

출처 schema와 기본 주기는 `cleany_logger.sources`, 기록 선택과 프로필은
`cleany_logger.recording`이 소유한다. 생산자는 이 schema로 ROS 구독을 설정할 수
있으며 logger_node가 직접 DDS discovery와 구독을 생성한다. 큰 원본의 선택은
기본 OFF이며 카메라, 점군과 voxel은 대용량이므로 필요한 토픽과 Hz를 선택한다.
기록 정책은 세션에, 큐 폐기 수는 metadata에 남기고 기존 파일을 자동 삭제하지 않는다.
