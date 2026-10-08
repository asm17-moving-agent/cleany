# Cleany — 미결정 사항과 다음 작업

기준일: 2026-10-09. 로봇 구현 브랜치 `feat/runtime-integration`, 소스 기준 `c082d1d`, KB commit `9931007`. 초기 문서 기준 `16504ee`에 runtime cleanup 6개 커밋과 Nav2 BT 추적·설정형 기록 기능을 통합했다. 10/09 accuracy review·SCRUM-426 도구·ROS 그래프 개선·navigation 회귀 도구·공유 voxel 변경도 통합했다. 기존 주행 기록과 이번 통합 검사를 구분한다.

## 확정하지 말아야 할 제품 질문

- 운영자 인터뷰·현장 관찰로 어떤 관리 부담과 가치를 검증할 것인가?
- 실제 데모에 둘 쓰레기·분실물 후보의 종류·개수와 좌석·초기 배치는 무엇인가?
- 분실물 발견 후 물리 행동·보관·운영자 인계는 무엇인가?
- 저신뢰·실패·미처리를 Dashboard에서 어떤 사용자 흐름으로 표시할 것인가?
- 전후 영상·관찰 자료의 보관 기간·접근 권한·삭제 기준은 무엇인가?
- 목표 E2E를 줄여야 할 경우 어떤 축소 데모를 팀이 승인할 것인가?

이 질문은 KB에 남아 있다. 분실물 자동 수거·성공률 숫자·영상 retention·담당자·일정을 ChatGPT가 만들어 확정하지 않는다.

## 기술적으로 아직 선택·검증할 경계

| 주제 | 현재 기준과 열린 부분 |
|---|---|
| Task Planner | 같은 typed proposal 경계에서 Rule-based/로컬 VLM/API VLM 비교. 최종 모델 미선정 |
| Perception | detector/mask/depth→Scene 품질·실패·ID·timestamp 계약을 실제 adapter로 연결해야 함 |
| AI 호출 | 행동 checkpoint에서 재판단. 고정 주기나 실행 중 trajectory 교체를 기본으로 삼지 않음 |
| Navigation tool | 현재 Mission이 Navigator 직접 호출. Planner가 Navigation까지 제안할지는 미정 |
| Manipulation | VLA·MoveIt·controller·fallback의 Skill별 조합, grasp/placement·낙하 검증 미완료 |
| 실제 접근점 | 시뮬 seat/home와 실환경 pose·팔 reach·dock는 별도 검증 |
| 사람·안전 | 사람 존재/접근의 거리·정지·재개 정책, 실제 힘·제동·e-stop 회로 검증 필요 |
| 전력·실장 | payload·배터리·fuse·배선·USB bandwidth·서보 배치 확정 필요 |
| 운영·복구 | transport는 구현됐지만 원격 reset/복귀 요청의 제품 안전 정책은 자동 확정하지 않음 |

## KB와 현재 구현이 다른 부분

| KB에 남은 설명 | 이번 코드 확인 | 해석 |
|---|---|---|
| 작업 후/행동별 재관찰 없음 | CleaningTree에 Reobserve·VerifyCompletion 있음 | 경계 logic은 구현됨. 실제 영상 인지는 mock |
| Backend/외부 adapter·재연결 없음 | Bridge/Gateway·outbox·snapshot·ACK 구현됨 | 현 통합 브랜치의 구현 상태를 사용 |
| 기본 PERCEIVE/PLAN/EXECUTE FSM | 별도 비동기 Runtime의 WORKING 안 BT | 구 enum/manager 경로는 cleanup에서 제거됨 |
| MCU 모델·통신 미정 | ESP32-S3 native USB micro-ROS 구현 | 하드웨어 실행 검증은 별도이며 실물 전체 완료를 뜻하지 않음 |
| Robot Interface 목표 구조 | 독립 package는 scaffold, 직접 port/adapter 존재 | 모든 경계가 하나의 Robot Interface 구현을 통과한다고 말하지 않음 |

KB의 목표 책임과 구현 사실을 구분해서 설명한다. 여기서 KB 원문을 고치거나 새 Decision을 만들지는 않았다. 일부 README의 옛 링크·정책 예시도 현 KB와 다르므로 완성된 제품 정책으로 사용하지 않는다.

## 다음 작업의 우선순위 제안

현재 작업 지시가 없다면 아래 흐름으로 제안한다. 이 순서는 팀이 승인한 일정이 아니다.

### 1. 목표 E2E에서 실제로 연결할 작업 경계 결정

mock 인지·Planner·executor 중 어느 하나를 먼저 실제 adapter로 대체할지 정한다. 한 행동마다 최신 Scene→검증된 proposal→결과→새 Scene이 연결돼야 한다. 완료 기준에는 입력·출력·실행 출처·실패·stale·취소를 포함한다. 기존 grasp plan-only를 ‘수거 Skill 완성’으로 연결하지 않는다.

### 2. 실제 조작 Skill의 단계와 결과 검증

접근→close→파지 확인→lift/운반→release→placement 확인→팔 회복을 분리한다. 현 데모는 접근까지다. 물체가 그리퍼에 남거나 놓인 위치가 불확실한 경우 SUCCESS·base 이동 허가를 주지 않는 결과 계약이 필요하다. 시뮬레이터 검증 뒤 실물 조건을 별도로 검토한다.

### 3. 주행의 정확도·반복성 확보

같은 시작 pose·map·센서·GUI·DB로 홈→43→홈→13→홈을 반복한다. 13번 홈 GT 오차 약 10–13cm를 wheel/AMCL/TF로 나누어 진단하고, SlowZone과 DWB 가속 명령의 약 0.01m/s creep를 분석한다. 실측 RTF 2 과제는 전체 안전·센서·물리 조건을 유지하는 성능 과제로 다룬다.

### 4. 실물 subsystem 증거 확보

공중 무부하 속도 시험 다음으로 하중·지면·slip·거리/yaw·정지 시간·통신 단절·e-stop을 측정한다. 초기 공중 시험 상한을 바닥 주행 허용값으로 사용하지 않는다. 그 후 좌석 접근·팔 workspace·물체 파지/투입의 실물 검증을 통합한다.

### 5. 문서·전체 검사·배포 상태 갱신

변경 package 검사를 실행하고 여러 경계를 바꾸면 전체 workspace 검사로 넓힌다. 초기 hand-eye 실패와 현재 상태를 분리해 확인한다. 원격 main/PR/CI, control-plane의 실제 사용 브랜치·계정·인증, Jetson 배포 상태는 이번 자료만으로 최신을 확정할 수 없다.

## ChatGPT 분석에 필요한 최소 데이터

ChatGPT는 계획·가설·판정 기준을 만들고 사용자가 실행한 결과를 받아 해석한다. 모든 파일을 처음부터 요구하지 않고 질문에 필요한 범위만 요청한다.

- 홈 오차: 같은 시각의 원래 목표·추정 pose·GT pose·wheel odometry·map→odom→base TF, 시작 조건.
- 반복 정지: Nav2 요청·Collision Monitor 이후·최종 cmd_vel·측정 속도·guard decision·센서 age.
- RTF: clock와 monotonic wall·CPU/GPU 부하·렌더러·physics step·센서 구독 조건.
- 조작: snapshot/object ID·후보·선택 팔·실행 단계·joint feedback·물체/placement 상태·stop/arm recovery.
- 인지/Planner: 입력 장면·mask/depth·frame/timestamp·structured 출력·실패 응답·지연·사용 모델.
