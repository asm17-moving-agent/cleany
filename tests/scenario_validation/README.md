# Gazebo 시나리오 반복 검증

## 범위·입력

- 시설 월드에서 대상 의자만 제거; 다른 의자 유지
- 이전 43/13 fixture: 누락 의자 두 개 먼저 복원
- 모델 대응: office_chair_NN / facility_chair_ROOMN
- 기반: Fortress·Nav2/AMCL·reactive safety·obstacle memory·센서·wheel odometry
- 책상 작업: mock. 실물·manipulation 완료 증거 아님
- 고정 gate: 도착 8 cm, freshness·속도·취소, navigation 180초
- Instance별 ROS domain=domain-base+i, 고유 partition·DB·출력
- 준비 완료 후 파일 barrier로 동시 시작; lifecycle manager 세 개 순차 STARTUP

## 실행

- 준비: Humble에서 `make build-scenario-runtime`
- 호출: 호스트. Container에서 동일 소스·데이터 절대 경로 접근 필수

```bash
python3 tests/scenario_validation/parallel.py \
  --root "$PWD" --container ros2-humble \
  --count 1 --name smoke-001 --seat seat-44 --sequence seat-44,seat-16 \
  --stop-on-failure --domain-base 120
```

| 경로 | 기본값 / 변경 옵션 |
|---|---|
| Data root | ../cleany-data / --data-root, CLEANY_DATA_ROOT |
| Fixture | fixtures/cad-fortress-command-hold / --fixture |
| 출력 | runs/scenarios/name / --output-root |
| Runtime overlay | builds/worktree/install-scenario-runtime/setup.bash / --runtime-overlay |
| Build root | CLEANY_BUILD_ROOT |
| AMCL | system / --amcl-overlay |
| Navigation | baseline / --navigation-overlay |
| 관측 overlay | --diagnostic-overlay |

- 기존 출력 덮어쓰기 금지. Fixture/overlay 누락은 실행 전 실패
- Container: 호출자 UID/GID; container에서도 해당 UID 유효 필수
- 좌석: --seats. 연속 왕복: --repeats. 재시작 없는 전환: --sequence
- Queue: `queue.py --root "$PWD" --jobs JOBS.json`; parallel 옵션의 JSON 객체 배열, boolean 지원
- Provenance: commit·branch·dirty·install prefix·소스/입력/바이너리 hash
- Git identity: 호스트 source.json 고정 → worker 전달

## Fixture·profile

- URI: repo://=checkout, fixture://=선택 fixture. 상대 file·로컬 Fuel cache 지원
- 과거 절대 경로 추측 치환 금지. 활성 URI 이관 manifest와 과거 hash 분리
- 후보 profile: profiles/cad_fortress_command_hold.yaml
- 내용: CAD Fortress yaw 보정·DWB 비용·발행 명령 0.2초 hold 충돌 검사
- 실물 보정값으로 사용 금지

```bash
python3 tests/scenario_validation/prepare_profile.py \
  /path/to/cleany-data/fixtures/baseline /path/to/cleany-data/fixtures/candidate \
  --profile tests/scenario_validation/profiles/cad_fortress_command_hold.yaml
```

- 생성 결과: --fixture + yaw 보정 지원 odometry + 현재 navigation overlay로 실행
- 기존 결과 덮어쓰기 금지. 설정 두 개 신규 생성; 나머지는 원본 symlink
- 원본 fixture 실행 중 고정. Profile/config·mesh/material/texture SHA-256 기록
- Fuel: 로컬 cache 파일 고정
- 누락·빈 asset·미지원 URI·geometry 로딩 오류: 미션 시작 금지
- Profile 생성·단위 테스트 통과 ≠ 50/60 달성

## 의자 전환

- 의자는 static group 밖에 유지; Gazebo create/remove로 이전 복원·다음 제거
- 매 미션 전 scene/info에서 전체 의자·전체 모델 대조
- 조건: 이전 의자 존재 + 대상 의자만 없음. Service ACK만으로 전환 인정 금지
- 원본 응답·hash: chair-scenes/, 시도 이력: chair-transitions/
- 응답 유실: 실제 장면 확인; 적용된 작업 반복 금지
- 미적용 작업·장면 조회: 최대 3회. Transport 3초 / CLI 6초
- 장면 조회 실패·다른 모델 변경: 시작 금지
- 전환은 fixture 작업. Navigation deadline·충돌·도착·freshness gate 유지
- 동적 의자 profile의 자원량과 immutable scene benchmark 구분

## 시나리오

| 이름 | 입력 / 확인 |
|---|---|
| normal | 정상 왕복 |
| cancel_outbound / cancel_return | 출발·복귀 취소 |
| safety_stop | synthetic E_STOP 전달·정지; 물리 producer 검증 아님 |
| sensor_loss | depth adapter 종료; 취소 전 실제 정지 확인 |
| obstacle / blocked | 장애물 / target 주변 폐쇄 |
| path_failure | 실제 planner process 유실 |

- 실패·정지는 원래 navigation 제한 내 판정
- 정상 성공과 기대한 실패 시나리오 통과 구분

## 결과·판정

- 원본: batch.json, resources.json, world manifest, trace.jsonl, result.json, shutdown.json, ROS/Gazebo 로그
- 최초 실패·자원 포화·준비 실패 보존
- Progress: 미션별 progress.json atomic 기록. 최종 근거: result.json + shutdown.json
- --stop-on-failure: 첫 정상 실패에서 중단; 요청 범위·stopped_on_failure 기록
- --failure-limit 11: 60회 중 50회 성공 불가능 시 중단. 첫 실패 옵션 우선
- `continuous_verdict.py WORKER --output FILE`: 6좌석×10 순서·의자·ID·왕복 결과·정지·종료 대조
- 부분 50건 실행·오래된 다른 mission 결과·명령만 0인 실제 이동: 통과 금지
- `inventory.py FIXTURE`: 전체 좌석 의자 조건
- `audit.py OUTPUT`: mission ID·예상 결과·cmd_vel/GT 정지·shutdown
- Harness 관찰 시간: 요청 수에 비례. 개별 navigation deadline 유지
- 종료: PID와 현재 실행 경로 재대조 후 cleanup
- Navigation overlay 재검증: `--navigation-overlay "$(make -s print-navigation-install-base)/setup.bash"`
- 실제 prefix·plugin/wheel odometry source hash 기록; 원래 실패와 수정 후 결과 분리

## 도구·과거 근거

- 단위 검사: make test-scenario-tools; 실제 Gazebo 검증과 별도
- 대표 좌석: `representatives.py RUNS --fixture FIXTURE --jobs JOBS.json`
- 보정 시도: --corrections. 원래 시도도 보존
- 과거 driver/job/profile: CLEANY_DATA_ROOT/experiments/legacy-20261010; 그대로 재실행 금지
- GPU: NVIDIA EGL 파일 존재 시만 선택. Mesa 설정 유지; GPU 도구 없으면 미측정
- 호스트별 처리량·RTF, fresh·continuous 결과 구분
- 원본 위치: [EVIDENCE](../../docs/EVIDENCE.md)
