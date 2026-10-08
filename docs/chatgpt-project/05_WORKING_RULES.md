# Cleany — ChatGPT 협업과 로컬 작업 규칙

기준일: 2026-10-08. 로봇 구현 브랜치 `feat/runtime-integration`, 소스 기준 `3f86ecb`, KB commit `9931007`. 초기 문서 기준 `16504ee`에 runtime cleanup 6개 커밋과 Nav2 BT 추적·설정형 기록 기능을 통합했다. 기존 주행 기록과 이번 통합 검사를 구분한다.

## 역할 분담

ChatGPT는 업로드된 정보로 제품·설계 설명, 가설·비교, 코드 검토, 변경안·검증 계획과 로컬 코딩 에이전트용 작업 지시를 작성한다. 사용자 또는 로컬 코딩 에이전트가 파일·환경·로봇에 접근해 실제 작업을 수행하고 결과를 제공한다.

경로·package·topic 이름은 구현 식별용이다. ChatGPT에게 ‘저장소를 열고 알아서 실행하라’는 절차로 사용하지 않는다. 기본 프로젝트 논의는 이 6개 본문으로 진행한다. 특정 코드의 실제 수정 판단에는 관련 소스와 실행 설정을 제공받는다.

## 로컬 작업 지시 템플릿

```text
목표: 어떤 사용자/로봇 동작을 바꿀 것인지
현재 근거: 제공된 코드·설정·시험과 그 한계
변경 범위: package/경계, 유지할 기존 동작
필수 조건: 안전·데이터·시간·frame·소유권 계약
구현: 필요한 adapter/계약/설정, 미정 정책의 처리
검증: 가장 가까운 동작/실패 검사, 추가 실행 환경
완료 기준: 결과·관찰·정지·복구 등 측정 가능한 조건
보고: 변경 요약, 환경/commit, 실제 결과와 미검증 항목
```

명령·파일 변경안을 제안했다는 것과 구현·시험을 완료했다는 것을 구분한다. 새 로그가 없으면 과거 시험 수치로 새 변경의 성공을 선언하지 않는다.

## 작업 경계

- 로컬 작업자는 저장소 AGENTS와 관련 package README를 확인한다. 구현 사실·interface·설정·실행법이 바뀌면 해당 README도 갱신한다.
- 제품·설계는 KB Planning/Technical, 선택 이유·대체 관계는 Decision, 미결정 사항은 Questions로 구분한다. Raw는 실험·초안이며 공식 결정이 아니다.
- KB submodule 내부는 명시 요청 없이는 수정하지 않는다. 검토 중인 전제를 코드에 확정 사실로 하드코딩하지 않는다.
- 미커밋 변경과 다른 워크트리·실행 프로세스의 소유권을 보존한다. 원본 브랜치를 임의 reset/clean하지 않는다.
- build/install/log·cache·대용량 asset·mesh는 임의 편집·재생성하지 않는다.
- Python core는 typed model·port와 ROS shell을 분리한다. 설정은 parameter/config로 명시하고 dependency 변경은 manifest/install 경계에 반영한다.
- 센서 freshness·정확한 timestamp TF·충돌·제동·속도/가속도·정지 확인·도착·timeout을 느슨하게 해 시험을 통과시키지 않는다.
- 운영 Dashboard와 읽기 전용 개발 모니터를 혼동하지 않는다. 모니터의 정상 표시가 주행 허가는 아니다.
- 물리 구동·firmware upload는 별도 명시 요청과 장치·시험 조건 확인이 필요하다. 이 인수인계가 실제 구동 승인에 해당하지 않는다.

## 환경과 실행의 의미

로봇 개발 표준은 Ubuntu 22.04 native ROS 2 Humble이며 기존 검증은 Fedora 44 호스트의 `ros2-humble` Distrobox에서 수행했다. Gazebo Fortress가 기준이고 Jazzy/Harmonic은 별도 호환 환경이다. Jetson 목표는 Orin NX 16GB·JetPack 6.2·Humble이며 호스트의 simulator 성능을 Jetson 성능으로 해석하지 않는다.

반복 build/test 진입점은 루트 Makefile이다. 로컬 작업자는 상태·overlay·DDS domain·실행 중 프로세스부터 확인한다. 서로 다른 실험에는 별도 DB·출력 디렉터리를 쓰고 복구 시험은 같은 journal을 유지한다. simulator/Nav2/bridge/pose producer와 canonical TF를 중복 실행하지 않는다.

아래는 로컬 실행자가 사용할 검사 후보다. ChatGPT가 실행할 수 있는 명령 목록이 아니다. 실제 변경 범위에 맞는 것만 선택한다.

| 명령 | 확인 범위 |
|---|---|
| `make test-mission-core` | 순수 FSM/BT/bridge 계약과 실패·영속성 |
| `make test-dev-monitor` | 개발 관측·API·기록 core |
| `make test-navigation` | navigation package와 C++ plugin 경계 |
| `make build-mission-sim` | bringup까지 의존 package build |
| `make build-dev-monitor` | 웹과 ROS 관측 package build |
| `make test-motor-core`, `make test-base` | MCU core/ROS base 경계, 실물 구동 아님 |
| `make test` | 전체 workspace build/test. 실패·skip를 별도 기록 |

시설 simulator의 기본 bringup 지도·목표는 study-cafe일 수 있어 full-facility 목표를 명시해야 한다. map ID와 target mapping을 맞추고 기존 Nav2가 있으면 runtime만 연결한다. 실물 초기 base profile은 바퀴를 띄운 시험용이며 실물 DDS domain과 mock domain도 구분한다.

운영 Dashboard/Backend의 기존 기본 포트는 8080, 개발 모니터는 8768이다. 이는 과거 실행 설정이며 현재 서비스가 열려 있다는 뜻이 아니다. 이번 자료는 배포 URL·인증·계정의 현재 운영 상태를 확인하지 않았다.

## 검증·보고 기준

변경에 가장 가까운 실제 동작·안전·실패·timestamp/TF·데이터 보존 검사를 먼저 실행한다. core 변경은 package pytest, manifest/launch/install은 build, 여러 package/interface 경계는 전체 검사를 고려한다. 소스 문자열·고정값을 그대로 확인하는 검사를 늘리는 것을 목표로 하지 않는다.

| 증거 | 주장할 수 있는 것 | 주장할 수 없는 것 |
|---|---|---|
| 코드·정적 검사 | 구현과 계약의 존재·정적 유효성 | runtime·물리 성공 |
| unit/mock ROS | 특정 성공/실패·transport 흐름 | 실제 인지·수거·제동 |
| Gazebo | 시뮬 주행·topic/TF·관측 조건 결과 | 실물 slip·접촉·안전 인증 |
| MuJoCo/MoveIt | 해당 조작 장면·경로·controller 실행 | 실물 grasp/placement 성공 |
| 기록 재생/UI | 저장된 자료의 표현과 관측 UI | 새 로봇 실행 |
| 실물 시험 | 기록한 장치·하중·전원·환경 범위의 동작 | 다른 조건과 전체 E2E |

보고에는 commit/환경/입력·설정, 성공·실패·skip, 요청 RTF와 실측 RTF, 추정 pose와 GT/실측 오차, 최종 cmd/측정 정지·IDLE·잔여 프로세스 및 한계를 포함한다. UI 변경은 실행 브라우저 화면을 확인한다. static build나 서버 시작만으로 시각 검증을 대체하지 않는다.

## 이 자료의 작성 근거와 제한

작성자는 KB의 Planning·Technical·Questions·주요 Decision과 구현 package README를 읽고, Runtime/node/BT/model/manipulation 경계, Bridge session, base core, grasp selector, navigation parameter 구성 및 상대 Backend gateway/DB 코드를 확인했다. 전체 코드·모든 실행 환경을 감사한 것은 아니다.

검증 수치는 기존 통합·reactive·연속성·모니터 기록과 ESP32 README의 실측 보고를 요약했다. 원본 소스·시계열·사진·DB·시험 장치는 이 6개에 포함되지 않는다. 기본 맥락 이해에는 외부 경로를 요구하지 않지만 새 원인 확정·패치·물리 안전 판단에는 해당 자료가 필요하다.
