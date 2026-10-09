# SCRUM-426 Gazebo 시나리오 반복 검증

시설 월드에서 대상 좌석의 의자만 제거하고 다른 의자는 유지한다. 이전 43/13 fixture의
두 누락 의자를 원본 study 월드에서 먼저 복원한다. 모델 대응은 시설 generator의
`office_chair_NN`, `facility_chair_ROOMN` 규칙을 따른다. 제거 전후 모델 비교와 월드
SHA-256을 실행 결과에 남긴다.

`instance.py`는 headless Fortress, 실측 `/clock`/wall RTF, Nav2/AMCL, reactive safety,
obstacle memory, 실제 센서 및 wheel odometry로 좌석 왕복을 수행한다. 책상 작업은 mock이다.
8 cm runtime 도착 게이트, freshness, 속도, 취소와 180초 navigation deadline은 변경하지 않는다.

`parallel.py`는 각 인스턴스에 ROS domain 80+i, 고유 Gazebo partition, DB와 출력 경로를
배정한다. 모든 인스턴스의 준비를 확인한 뒤 같은 파일 barrier로 미션을 시작한다.
Nav2의 세 lifecycle manager는 discovery 후 순서대로 STARTUP을 요청한다.

저장소 루트에서 ROS Humble 환경으로 `make build-scenario-runtime`을 실행한 다음,
호스트에서 실행한다(container는 동일한 소스·데이터 절대 경로를 볼 수 있어야 한다).

```bash
python3 tools/scenario_validation/parallel.py \
  --root "$PWD" --container ros2-humble \
  --count 1 --name smoke-001 --seat seat-44 --sequence seat-44,seat-16 \
  --stop-on-failure --domain-base 120
```

`--data-root` 또는 `CLEANY_DATA_ROOT`의 기본값은 worktree 옆 `cleany-data`다.
출력은 `runs/scenarios/<name>`, 기본 fixture는 `fixtures/cad-fortress-command-hold`,
runtime overlay는 `builds/<worktree-name>/install-scenario-runtime/setup.bash`다.
`CLEANY_BUILD_ROOT`, `--runtime-overlay`, `--fixture`, `--output-root`로 각각 변경한다.
기존 출력은 덮어쓰지 않고 fixture/overlay가 없으면 실행 전에 오류로 종료한다.
`--amcl-overlay`를 생략하면 system AMCL을 사용한다. 실제 commit/branch/dirty 상태,
설치 prefix, 소스·입력·바이너리 해시를 결과에 기록한다. Git identity는 호스트에서
`source.json`으로 고정하고 worker에 전달하므로 container Git 버전에 의존하지 않는다.

`--seats seat-01,seat-a1-01`로 서로 다른 대상 월드를 지정하고, `--repeats N`으로 같은
인스턴스와 의자 조건에서 연속 왕복한다. container는 호출자 UID/GID로 실행해 결과를
호스트에서 읽을 수 있게 한다. 해당 UID가 container에서도 유효해야 한다.
`queue.py --root "$PWD" --jobs JOBS.json`도 같은 데이터·출력 옵션을 받는다.
이름/count/seat 등 `parallel.py` 옵션을 JSON 객체 배열로 전달하며 boolean 옵션도 지원한다.

fixture의 `repo://ros2_ws/...`는 현재 checkout, `fixture://...`는 선택한 fixture를 기준으로
해석한다. 상대 file 경로와 이미 캐시된 Fuel도 지원한다. 과거 호스트 경로를 추측해서
치환하지 않는다. 이관한 활성 fixture는 외부에 별도 URI 변환 manifest를 갖고,
과거 검증 원본과 원래 hash는 변경하지 않는다.

현재 연속 60회 검증 후보는 `profiles/cad_fortress_command_hold.yaml`에 보존했다.
CAD Fortress 전용 wheel yaw 보정과 DWB 경로 진행/중앙 장애물 비용에 더해,
실제 발행 명령을 0.2초 유지하는 경우의 충돌 검사도 적용한다.
물리 로봇 보정값으로 사용하지 않는다. 기본 fixture로부터 다음처럼 재현한다.

```bash
python3 tools/scenario_validation/prepare_profile.py \
  /path/to/cleany-data/fixtures/baseline /path/to/cleany-data/fixtures/candidate \
  --profile tools/scenario_validation/profiles/cad_fortress_command_hold.yaml
```

생성한 경로를 `parallel.py --fixture`에 전달하고, yaw 보정을 지원하는 odometry와
현재 navigation overlay를 지정한다. `prepare_profile.py`는 기존 결과를 덮어쓰지 않는다.
준비 단계와 worker 시작 시 실제 mesh/material/texture 파일을 검사하고 SHA-256을
기록한다. Fuel 모델은 기존 로컬 캐시의 파일로 고정한다. 누락·빈 파일·지원하지 않는
URI 또는 Gazebo 시작 로그의 geometry 로딩 오류가 있으면 미션을 시작하지 않는다.
두 설정 파일은 새 파일로 만들고 지도·월드·나머지 설정은 원본을 가리키는 symlink로
공유한다. 원본 fixture는 실행 중 고정해야 하며 profile/config 해시는 manifest에 남긴다.
이 프로필의 정상 왕복 성공률은 [검증 색인](../../docs/validation/README.md)에 안내한 실행 원본로
판정한다. fixture 생성이나 도구 테스트 통과만으로 50/60 달성을 인정하지 않는다.

결과 디렉터리에는 batch.json, 자원 시계열 resources.json, 인스턴스별 world manifest,
trace.jsonl, mission result.json, shutdown.json과 ROS/Gazebo 로그를 저장한다.
자원 포화/준비 실패/미션 실패를 성공한 실행과 따로 보고하며 최초 실패를 보존한다.
실제 로봇 또는 manipulation 완료 증거로 사용하지 않는다.

시나리오: `normal`, `cancel_outbound`, `cancel_return`, `safety_stop`, `sensor_loss`,
`obstacle`, `blocked`, `path_failure`. `safety_stop`은 synthetic E_STOP 알림으로
runtime/Nav2 정지 전달을 검사하며 물리 e-stop producer를 검증하지 않는다.
`sensor_loss`는 depth adapter를 종료하고 취소 전에 실제 정지를 관찰한다.
`path_failure`는 실제 planner process 유실, `blocked`는 target 주변의 물리적 폐쇄를
입력으로 사용한다. 원래 navigation 제한 안에서 실패하거나 정지해야 한다.

`--sequence seat-43,seat-13`은 재시작 없이 좌석을 바꾼다. 이 경우 의자 모델을 static
group 밖에 유지해 Gazebo create/remove service로 이전 의자를 복원하고 다음 대상
의자를 제거한다. 이 profile의 자원 사용량은 immutable scene benchmark와 구분한다.
연속 시험은 매 미션 전에 실제 `/world/.../scene/info` 응답에서 전체 의자 모델을
대조한다. 이전 의자는 존재하고 대상 의자만 없어야 진행하며, 원본 protobuf 응답과
해시를 `chair-scenes/`와 결과에 남긴다. 생성/삭제 service ACK만으로 전환을 인정하지 않는다.
전환 전후에는 전체 모델 이름도 대조한다. 서비스 응답이 유실되면 실제 장면을 읽어
이미 적용된 작업은 반복하지 않고, 미적용된 작업만 최대 세 번 재시도한다.
각 요청의 3초 transport timeout과 6초 CLI 제한은 그대로다. 장면 읽기도 최대 세 번이며,
읽을 수 없거나 다른 모델이 바뀌면 미션을 시작하지 않는다. 모든 시도와 원본 장면은
`chair-transitions/`에 남긴다. 이는 미션 사이의 fixture 전환이며 navigation deadline,
충돌·도착·freshness 기준이나 미션 실패 판정은 변경하지 않는다.

개선 중에는 `--stop-on-failure`로 첫 정상 미션 실패에서 순서를 중단한다.
요청 개수는 그대로 기록하고 `stopped_on_failure`에 실패 위치를 남기므로,
조기 중단된 시험을 전체 순서 통과로 판정하지 않는다. `--fixture`로 별도 비교 입력,
`--diagnostic-overlay`로 관측용 overlay를 선택할 수 있다. 지정 경로는 container
실행 전 절대 경로로 바꾸며 기존 fixture를 덮어쓰지 않는다. wheel odometry의
실제 Python 소스 hash도 결과에 기록한다.
완료한 각 미션은 `progress.json`에 원자적으로 기록하며 종료 후 `result.json`과
`shutdown.json`을 최종 근거로 사용한다. 60회 승인 시험에서는 `--failure-limit 11`로
50회 성공이 불가능해지는 시점에 중단할 수 있다. 첫 실패 중단 옵션이 우선한다.
`continuous_verdict.py WORKER --output FILE`은 원래 6좌석 × 10 순서, 의자 전환,
미션 ID, 목적지/복귀 결과와 정지/종료를 대조하고 50/60 목표를 판정한다.
실패 목록은 보존하며 50건만 실행한 부분 결과는 통과시키지 않는다.

`inventory.py FIXTURE`는 모든 좌석의 의자 조건을 검사한다. `audit.py OUTPUT`은
최종 mission ID, 예상 결과, 실제 cmd_vel/GT 정지와 shutdown을 확인한다.
기대된 실패 시나리오의 통과와 정상 미션 성공은 다른 결과다. 정상 성공 판정에서
다른 미션의 오래된 결과나 명령만 0이고 실제로 움직이는 상태는 거부한다.

자동 판정은 요청한 미션 개수까지 확인한다. 연속 좌석 시나리오의 harness 관찰 제한은
전체 요청 수에 비례하며 개별 runtime navigation deadline은 그대로 유지한다.
종료 시 기록된 PID가 현재도 해당 실행 경로의 프로세스인지 다시 확인한다.

도구의 최소 관련 검사는 ROS를 시작하지 않고 `make test-scenario-tools`로 실행한다.
planner 수정 재검증에는 `--navigation-overlay "$(make -s print-navigation-install-base)/setup.bash"`로
해당 워크트리에서 빌드·테스트한 navigation을 기준 runtime 위에 적용한다. 지정하지
않으면 기존 baseline을 사용한다. 결과에는 실제 선택된 navigation install prefix와
공유 라이브러리 SHA-256도 남겨 원래 실패와 수정 후 실행을 구분한다.
월드 보존, 미션 결과와 정지 판정, 불완전한 반복 거부, 처리량 중앙값 선택,
현재 프로세스 소유권 검사를 확인한다. 이는 실제 Gazebo 시나리오 검증과 별개다.

과거 survey/after_survey/after_smoke_recheck/repeat_representatives driver,
고정 job 목록과 이전 accuracy 후보는 `$CLEANY_DATA_ROOT/experiments/legacy-20261010/`에
당시 공통 모듈·테스트와 함께 보관했다. 현재 반복 검증에는 위 queue/parallel과
명시적인 job 파일을 사용한다. 보관 사본은 과거 절대 경로를 포함하므로 그대로 재실행하지 않는다.

NVIDIA EGL vendor 파일은 존재할 때만 선택하고 Mesa는 실행 환경 설정을 유지한다.
NVIDIA utilization 도구가 없는 호스트의 GPU 자원 값은 미측정으로 남긴다.
호스트별 처리량/RTF와 fresh/continuous 결과는 분리한다.

`representatives.py RUNS --fixture FIXTURE --jobs JOBS.json`은 외부 fixture와
명시한 전체 좌석 조사 순서에서 대표 좌석을 고른다. 보정 시도는 `--corrections`로
추가하며 원래 시도도 보존한다.
