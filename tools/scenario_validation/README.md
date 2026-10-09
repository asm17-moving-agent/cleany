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

현재 데스크탑 환경에서 실행:

```bash
python3 tools/scenario_validation/parallel.py \
  --root /home/donglee/workspace/cleany/scrum-426-scenarios \
  --count 2 --name capacity-2-r2 --seat seat-43
```

`--seats seat-01,seat-a1-01`로 서로 다른 대상 월드를 지정하고, `--repeats N`으로 같은
인스턴스와 의자 조건에서 연속 왕복한다. 기존 결과 경로는 덮어쓰지 않는다.

데스크탑의 `cleany-ros-humble-cdi` container와 정확도 개선 commit 076b893의 설치 overlay,
Nav2 AMCL 1.1.20 overlay를 사용한다. 기준 입력은 `artifacts/scenarios/fixture`의
world.sdf, world.study.sdf, facility_integration 지도, targets.yaml, config이다.
외부 assets 경로는 기존 검증 fixture를 이식한 실행 환경 경로이며 결과 월드와 manifest에
남긴다. 도구 이식 시 설치 overlay 및 fixture 준비가 필요하다.

현재 연속 60회 검증 후보는 `profiles/cad_fortress_command_hold.yaml`에 보존했다.
CAD Fortress 전용 wheel yaw 보정과 DWB 경로 진행/중앙 장애물 비용에 더해,
실제 발행 명령을 0.2초 유지하는 경우의 충돌 검사도 적용한다.
물리 로봇 보정값으로 사용하지 않는다. 기본 fixture로부터 다음처럼 재현한다.

```bash
python3 tools/scenario_validation/prepare_profile.py \
  artifacts/scenarios/fixture artifacts/scenarios/cad-fortress-command-hold \
  --profile tools/scenario_validation/profiles/cad_fortress_command_hold.yaml
```

생성한 경로를 `parallel.py --fixture`에 전달하고, yaw 보정을 지원하는 odometry와
현재 navigation overlay를 지정한다. `prepare_profile.py`는 기존 결과를 덮어쓰지 않는다.
준비 단계와 worker 시작 시 실제 mesh/material/texture 파일을 검사하고 SHA-256을
기록한다. Fuel 모델은 기존 로컬 캐시의 파일로 고정한다. 누락·빈 파일·지원하지 않는
URI 또는 Gazebo 시작 로그의 geometry 로딩 오류가 있으면 미션을 시작하지 않는다.
두 설정 파일은 새 파일로 만들고 지도·월드·나머지 설정은 원본을 가리키는 symlink로
공유한다. 원본 fixture는 실행 중 고정해야 하며 profile/config 해시는 manifest에 남긴다.
이 프로필의 정상 왕복 성공률은 `docs/validation/scrum-426-scenarios.md`의 실행 결과로
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

`survey.py --root ROOT --wait-pid PID`가 기존 capacity queue의 실제 프로세스 identity를
확인하고 종료를 기다린다. 모든 선행 작업의 terminal status를 확인한 뒤 2/3개 구성의
긴 왕복을 각각 총 3회 비교한다. 정상 완료 횟수 / batch 전체 시간의 중앙값이 높은
안정 구성을 선택하고, inventory의 모든 좌석을 빠짐없이 독립 월드에서 검사한다.
처리량에는 준비와 종료 비용도 포함한다. survey 선택 근거와 전체 job 목록을 저장한다.

자동 판정은 요청한 미션 개수까지 확인한다. 연속 좌석 시나리오의 harness 관찰 제한은
전체 요청 수에 비례하며 개별 runtime navigation deadline은 그대로 유지한다.
종료 시 기록된 PID가 현재도 해당 실행 경로의 프로세스인지 다시 확인한다.

도구의 최소 관련 검사는 ROS를 시작하지 않고 `make test-scenario-tools`로 실행한다.
planner 수정 재검증에는 `--navigation-overlay ROOT/ros2_ws/install-navigation/setup.bash`로
해당 워크트리에서 빌드·테스트한 navigation을 기준 runtime 위에 적용한다. 지정하지
않으면 기존 baseline을 사용한다. 결과에는 실제 선택된 navigation install prefix와
공유 라이브러리 SHA-256도 남겨 원래 실패와 수정 후 실행을 구분한다.
월드 보존, 미션 결과와 정지 판정, 불완전한 반복 거부, 처리량 중앙값 선택,
현재 프로세스 소유권 검사를 확인한다. 이는 실제 Gazebo 시나리오 검증과 별개다.

`scenario_smoke_jobs.json`는 각 비정상 조건의 실제 실행을 한 번씩 확인할 입력이다.
정상 좌석 inventory 조사와 별도로 실행하고, smoke의 raw 판정이 기대와 다르면 먼저
원인을 고친 뒤 같은 조건의 반복 검증을 수행한다. 우회 장애물은 몸체 yaw가 아니라
직전 ground-truth 이동 방향에 배치한다. Mecanum의 측방 이동에서도 실제 진행 경로를
막는 자극을 만들기 위한 조건이다.

`after_survey.py --root ROOT --wait-pid PID`는 실제 survey process 종료와 모든 batch의
terminal 기록을 확인하고, 전체 좌석 inventory와 raw 실행 결과를 대조한다. 정상 실패는
성공으로 집계하지 않고 좌석별로 남긴다. 그 다음 7개 smoke 입력을 순서대로 실행하고
실제 자극, 미션 결과 및 정지 증거를 판정한다. smoke에서 기대와 다른 결과가 있으면
원인을 분석한 뒤 반복 입력을 구성한다. 현재 자동화 자체는 goal 완료를 선언하지 않는다.

`after_smoke_recheck.py --root ROOT --wait-pid PID`는 실제 baseline smoke driver의
종료를 기다리고 전체 좌석 coverage와 smoke queue/observer/launch 종료를 확인한다.
그 뒤 `make test-navigation`으로 독립 overlay를 빌드·검증하고, 1번 단일 실행,
원래 실패한 전체 좌석의 3개 이하 병렬 실행, 13/43번 대조 왕복을 수행한다.
기존 baseline 실행 및 결과는 유지하며 수정 후 결과를 별도 디렉터리에 남긴다.
이 단계도 정상 실패를 성공으로 바꾸거나 전체 반복 검증 완료를 선언하지 않는다.

`repeat_representatives.py --root ROOT --wait-pid PID --runs 10`은 navigation 재검증의
실제 종료와 모든 observer/launch 종료를 확인한 뒤 시작한다. baseline과 수정 후 시도는
모두 보존하고 명시된 correction queue 순서의 마지막 좌석 결과로 대표 6개를 고른다.
2개 좌석 전환 smoke 후 각 대표 좌석을 새 인스턴스에서 10회씩 검증하고, smoke가
통과하면 한 인스턴스에서 6개 좌석 순서를 10회 반복한다. 새 인스턴스 시험은 최대
3개 병렬, 연속 시험은 의자 모델을 개별 유지하는 1개 인스턴스다. 두 profile의 RTF와
처리량을 동일한 자원 조건의 인과 비교로 해석하지 않는다. 연속 smoke 실패 시 fresh
결과를 보존하고 연속 반복은 원인 분석 전까지 시작하지 않는다. 비정상 시나리오의
5회 반복은 별도이며, 이 driver가 목표 완료를 선언하지 않는다.

다른 호스트에서는 `make build-scenario-runtime`으로 Gazebo/mission/odometry를 별도
`ros2_ws/install-scenario-runtime`에 빌드한 뒤 `parallel.py --container ros2-humble
--runtime-overlay ROOT/ros2_ws/install-scenario-runtime/setup.bash --domain-base 120`을
사용한다. custom AMCL이 필요하면 `--amcl-overlay`로 명시한다. 없으면 system AMCL을
사용하므로 실제 package prefix와 호스트/architecture를 결과에 기록한다. NVIDIA EGL
vendor 파일은 존재할 때만 선택하고 Mesa는 해당 환경의 설정을 유지한다. NVIDIA
utilization monitor가 없는 호스트의 GPU 자원 값은 미측정으로 남긴다. 호스트별
처리량/RTF는 분리하며 local/desktop의 값으로 같은 하드웨어 비교를 만들지 않는다.
