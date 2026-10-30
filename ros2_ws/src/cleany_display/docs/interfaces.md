# Cleany 디스플레이

`cleany_display`는 ROS 상태를 구독해 로컬 HTTP 화면에 전달한다. 로봇 제어 토픽을 발행하거나 서비스와 Action을 호출하지 않는다. 토픽 이름과 화면 정책은 `config/display.json`, DDS 환경은 별도 설치 폴더의 `runtime.env`로 설정한다.

| 기본 구독 토픽 | 메시지 | QoS | 표시 의미 |
|---|---|---|---|
| `/mission/status` | `cleany_interfaces/msg/MissionStatus` | reliable, transient-local, depth 1 | IDLE → 대기, NAVIGATE_TO_TARGET/RETURN_HOME → 이동, WORKING → 확인, ExecuteOne 단계 → 수거, ERROR → 도움 요청 |
| `/mission/result` | `cleany_interfaces/msg/MissionResult` | reliable, transient-local, depth 1 | 실행을 관측한 미션의 SUCCESS/PARTIAL_SUCCESS → 완료, 검토 필요 → 도움 요청 |
| `/manipulation/execution_events` | `cleany_interfaces/msg/ManipulationExecutionRecord` | reliable, transient-local, depth 1 | ACTIVE → 수거 또는 준비 단계 확인, human_confirmation_required → 도움 요청, SUCCESS → 수거 완료 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | sensor data | 선속도 크기 0.02 m/s 또는 각속도 절댓값 0.05 rad/s 초과 → 이동, 양의 z 각속도 → 왼쪽 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | sensor data | CHARGING(1) → 충전, 유효한 percentage(0–1)만 백분율 표시 |
| `/cleany/display/state` | `std_msgs/msg/String` | sensor data | JSON으로 제한 시간 동안 화면 상태만 지정 |

충전 표시에는 실제 `BatteryState` 발행자가 필요하다. 이 패키지는 배터리 드라이버를 제공하지 않는다. 수거 실행 기록의 확인 단계는 기본적으로 ValidateRequest, EnsureReady, CaptureScene, SelectArmAndPath, ValidatePlan이며 `checking_substages`로 설정한다.

표시 우선순위는 미션 오류/사람 확인 필요, 제한 시간 지정 화면, 충전, 진행 중 수거, 미션 진행, 측정된 이동, 완료 순서다. 이벤트 방식의 상태는 발행자가 존재하는 동안 유지한다. 주행 측정값은 1.5초, 배터리는 15초 후 만료한다. 완료는 6초 동안 표시하며 이전에 저장된 수거 성공은 timestamp로 걸러낸다. `MissionResult`는 timestamp가 없으므로 가장 최근 실행을 관측한 미션의 결과만 완료로 표시한다.

처음 켰을 때 ROS 입력이 없으면 대기 얼굴을 표시한다. ROS 입력이 있었으나 모든 유효 입력이 사라지면 3초 후 도움 요청과 “연결을 확인해 주세요”를 표시한다. 로컬 HTTP 연결이 끊어져도 브라우저에서 같은 문구를 표시하고 재연결한다. 개별 입력이 없어도 다른 유효 입력이 있으면 그 상태를 사용한다.

## 제한 시간 화면 지정

```json
{"mode":"charging","duration_seconds":5,"direction":"forward"}
```

`mode`는 idle, moving, checking, collecting, complete, charging, help 중 하나다. `duration_seconds`는 기본 5초, 최대 60초이며 `direction`은 left, forward, right 중 하나다. 선택적인 `message`는 최대 120자의 일반 텍스트로 표시한다. 모드와 시간이 유효하지 않은 요청은 무시한다. 이 입력이 만료되면 관측된 로봇 상태로 돌아간다. 미션 오류와 사람 확인 필요 상태를 가리지 않는다.

## HTTP

기본 주소는 `http://127.0.0.1:8770`이다. `/`는 디스플레이, `/api/state`와 `/healthz`는 읽기 전용 JSON snapshot이다. `mode`, `direction`, `message`, `battery_percent`, `collected_count`, `connected`, `source`, `sources`, `poll_ms`, `rendering`을 반환한다. 알 수 없는 수치는 null이며 데모 값으로 대체하지 않는다. HTTP는 로봇 명령을 받지 않는다.

화면은 16:9 영역을 화면 가운데 맞추고 남는 영역은 어두운 배경으로 채운다. 상태가 반복 수신되어도 애니메이션을 처음부터 다시 시작하지 않는다. 대기 중에는 10초 간격으로 웃기, 둘러보기, 끄덕이기를 반복한다. 충전 중에는 Zzz, 수거 중에는 땀방울을 표시한다. 확인과 수거 동작은 3.2초, 도움 요청은 3.6초를 한 주기로 반복한다.

## 렌더링 설정

| 설정 | 기본값 | 브라우저 적용 범위 | 의미 |
|---|---|---|---|
| `rendering.max_canvas_width` | 1280 | 320–3840 | 얼굴 비트맵의 최대 가로 픽셀 수 |
| `rendering.max_pixel_ratio` | 2 | 1–2 | 장치 픽셀 배율의 상한 |
| `rendering.max_fps` | 60 | 15–60 | 얼굴의 초당 렌더링 횟수 상한 |

기본값은 60fps이고 캔버스는 최대 1280×720이며 화면 크기에 맞춰 확대한다. 안내 글자는 DOM으로 원래 해상도에서 표시한다. 장치의 출력 해상도와 주사율은 별개다. 애니메이션 시간은 렌더링 간격으로 계산하므로 프레임 수를 낮춰도 동작 주기는 유지된다. 상태 변경과 크기 변경 시에는 즉시 다시 그린다.

설정은 상태 응답에 포함되며 서버 재시작 후 브라우저 재실행 없이 적용된다. 브라우저의 requestAnimationFrame 콜백 수와 실제 캔버스 렌더링 횟수는 다르므로 성능 측정에서 구분한다.
