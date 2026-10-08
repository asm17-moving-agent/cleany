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

`inventory.py FIXTURE`는 모든 좌석의 의자 조건을 검사한다. `audit.py OUTPUT`은
최종 mission ID, 예상 결과, 실제 cmd_vel/GT 정지와 shutdown을 확인한다.
기대된 실패 시나리오의 통과와 정상 미션 성공은 다른 결과다. 정상 성공 판정에서
다른 미션의 오래된 결과나 명령만 0이고 실제로 움직이는 상태는 거부한다.
