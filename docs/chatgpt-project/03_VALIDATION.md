# Cleany — 구현 수준과 기존 검증 결과

기준일: 2026-10-09. 로봇 구현 브랜치 `feat/runtime-integration`, 소스 기준 `c082d1d`, KB commit `9931007`. 초기 문서 기준 `16504ee`에 runtime cleanup 6개 커밋과 Nav2 BT 추적·설정형 기록 기능을 통합했다. 10/09 accuracy review·SCRUM-426 도구·ROS 그래프 개선·navigation 회귀 도구·공유 voxel 변경도 통합했다. 기존 주행 기록과 이번 통합 검사를 구분한다.

## 현재 구현 수준을 먼저 구분하기

| 영역 | 이번에 코드·문서로 확인한 범위 | 완료로 볼 수 없는 범위 |
|---|---|---|
| 미션·Gateway | 실제 요청/수락/취소/영속 보고·재연결 경계 | 실물 청소 E2E |
| 자율주행 | 시설 Gazebo/Nav2 왕복과 안전·도착 검사 | 실제 지면 오차·사각지대·제동 |
| 책상 BT | 제안 하나→검증→실행→재관찰 logic | 실제 Perception·AI Planner 동작 |
| Grasp·MoveIt | 후보 생성·plan-only 검증·접근 데모 구현 | close/lift/운반/투입 전체 완료 |
| Base | ROS/micro-ROS/MCU 구현·공중 속도 실측 기록 | 지면 하중·정지 거리·완성 로봇 통합 |
| 개발 UI | 읽기 전용 관측·기록·브라우저 검증 | 운영 권한·물리 안전 판정 |

## 실제로 확인된 결과와 한계

아래는 기존 기록의 결과이며 이번에 재실행하지 않았다. 시점과 조건이 다른 실험을 하나의 평균이나 성공률로 합치지 않는다.

### 최신 reactive GUI 비교 — 2026-10-04

동일 GUI 조건에서 변경 전후 각각 1회, 새 프로세스·DB로 **홈→43→홈→13→홈**을 수행했다. Fortress/Ogre2, physics step 약 6.67ms, 요청 RTF 4 조건이다.

| 항목 | 변경 전 | 변경 후 |
|---|---:|---:|
| 43번 왕복 wall 시간 | 31.98초 | 21.41초 |
| 13번 왕복 wall 시간 | 110.65초 | 89.30초 |
| 13번 복귀 정지 수준 시간 | 9.57초 | 2.15초 |
| 실측 mission RTF (43 / 13) | 1.429 / 1.484 | 1.447 / 1.434 |
| 13번 홈 GT 위치 오차 | 11.48cm | 12.32cm |
| 미션 왕복 결과 | 모두 SUCCESS | 모두 SUCCESS |

GT는 simulator ground truth다. 미션 성공은 추정 pose 기반이므로 GT 오차와 구분해야 한다. 실측 2배속 목표와 홈 위치 정확도 문제는 남았다. 한 쌍의 성공으로 반복 신뢰성을 입증한 것은 아니다.

관련 navigation package 155 tests와 별도 ROS 장애/복구 27개 경우가 통과했다. 종료 시 최종 `/cmd_vel`·GT twist 0, runtime IDLE 및 관련 프로세스 정리를 기록했다. 추가 작은 창 실행에서도 RTF는 약 1.45–1.47에 그쳤다.

### 개발 관제보드 — 2026-10-04~05

자율주행을 주행 개요, 장애물·안전, 계획·제어, 센서·위치, 주행 비교의 5개 탭으로 확장했다. 실제 BT 로그, DWB 후보, 지도/costmap, 명령/실측 속도와 기록 재생을 연결했다.

- ROS/core 18개, 웹 6개 통과와 웹·관련 ROS 4개 패키지 빌드를 기록했다. 환경 지정형 웹 검사 2개는 미실행이다.
- 실제 Gazebo 43/13 왕복 SUCCESS 관측 및 별도 43 왕복 검증이 있다.
- 최종 탭 구분 수정은 같은 주행 기록을 HTTP/SQLite API로 재생해 검증했다. 새 주행 시험으로 계산하지 않는다. 관련 Playwright 2개가 통과했다.
- 시뮬 초기화의 비정상 측정 속도 379.586m/s는 원본 기록과 UI에 보존했다. 정상값으로 치환하지 않았다.
- 센서 stamp의 TF가 없으면 해당 레이어를 표시하지 않는다. 미수신·지연·미제공을 정상/안전으로 추정하지 않는다.

### 전체 통합 검사 상태

초기 전체 workspace build는 17 packages 성공이다. hand-eye 제외 검사에서는 359 tests, 0 errors/failures, 11 opt-in skipped를 기록했다. 당시 전체 `make test`는 631 tests 중 1 failure였고, hand-eye MuJoCo 단일 pose의 ChArUco corner 검출 부족이었다. 이를 최신 HEAD 전체 CI 통과로 해석하지 않는다.

## 앞선 개선 단계에서 확인한 내용

### 몸체 자기 관측 필터

초기 시설 주행에서는 depth 누적 voxel이 로봇 footprint 내부에 남아 복귀를 방해했다. sensor→hit 선분과 고정 몸체 OBB의 교차를 검사하여 몸체를 통과한 잘못된 hit와 clearing ray를 함께 버리도록 수정했다. 외부 장애물과 기존 안전 게이트는 유지했다. 수정 후 대시보드 요청→좌석 도착→모의 청소→홈 복귀를 두 번 연속 완료하고 최종 속도 0을 확인했다. 이 결과는 실물 검증이 아니다.

### 메시지 변환·TF 처리와 렌더링 부하

장애물 메모리의 TF blocking wait를 센서 원본 시각의 nonblocking 조회와 후속 재시도로 바꿨다. 대형 격자와 PointCloud2 변환을 typed array로 바꾸고, 구독자 없는 RGB bridge는 lazy로 변경했다. depth/CameraInfo/LiDAR 구독과 freshness 기준은 유지했다.

같은 홈 자세와 새 DB로 시작한 전후 비교에서 43번 왕복은 178.48→32.42초, 13번은 501.01→130.03초였다. 실측 RTF는 약 0.39에서 1.48–1.53으로 증가했다. 후 조건에는 physics step 약 6.67ms와 요청 RTF 4가 포함되므로 메시지 최적화 하나의 효과로 해석하지 않는다. 13번 홈 GT 오차는 13.09cm였고 실측 2배속은 달성하지 못했다.

Ogre1 시험은 약 2.03배를 냈지만 LiDAR 유래 occupied voxel이 footprint 내부에 생겨 경로를 거부했다. 유효한 2배속 주행으로 채택하지 않았다. GPU 주파수 하한 임시 조정도 효과를 입증하지 못했고 복원했다.

### 주행 연속성 개선

측정/직전 선택 이동 방향을 유지하는 속도 후보와 작은 반복 방향 변경을 줄이는 soft switching cost를 추가했다. 후보는 기존 가속도·충돌 검사를 통과해야 한다. 전체 body sweep의 depth 관측 진단은 실패 구간이 있어 shadow 전용으로 유지했다.

같은 GUI·시설·센서 조건에서 변경 전후 각각 2회 실행한 평균은 다음과 같다. 이 비교는 이후 reactive 비교와 별도 실험이다.

| 항목 | 개선 전 평균 | 개선 후 평균 |
|---|---:|---:|
| 43번 왕복 시간 | 36.5초 | 30.0초 |
| 13번 왕복 시간 | 157.2초 | 121.4초 |
| 13번 복귀 정지 시간 | 29.0초 | 12.9초 |
| 43번 홈 GT 오차 | 7.5cm | 8.5cm |
| 13번 홈 GT 오차 | 12.3cm | 10.9cm |
| 실측 RTF 범위 | 1.24–1.40 | 1.37–1.41 |

모든 해당 미션은 SUCCESS였지만 작은 표본이며 구간별 편차와 정확도 악화가 있다. navigation 142 tests가 통과했다. 이후 reactive 단계의 155 tests와 중복 합산하지 않는다.

## 물리 베이스에 대해 보고된 시험

ESP32 구현 README에는 2026-10-02 네 바퀴를 공중에 띄운 상태로 동시에 정·역 PWM 스윕하여 PI+feed-forward 초기값을 추정한 기록이 있다. 보정 후 정·역 0.5/2/5/10rad/s를 각각 5초 검증했고 마지막 0.5초 평균 속도 오차가 모든 휠에서 4.5% 이내였다고 보고한다. 최종 구동 비허가·PWM 0·fault 0을 기록했다.

이는 문서에 보고된 특정 전원·공중 무부하 실측이다. 이번에 시험 원본이나 장치를 재검증하지 않았다. 지면 하중·Mecanum slip·거리/yaw·USB loss 정지 시간·물리 e-stop·전체 로봇 E2E를 증명하지 않는다. 따라서 ‘실물 증거가 전혀 없다’와 ‘완성 로봇이 검증됐다’ 모두 부정확하다.

## 조작과 calibration 증거의 제한

Grasp selector·MuJoCo box/can 접근 데모와 hand-eye 기록/solver 코드는 존재한다. 이번 문서 작성에서는 데모를 실행하거나 새 성공률을 측정하지 않았다. can 데모는 pregrasp와 열린 gripper 접근까지이며 수거 성공 기록으로 포함하지 않는다. 초기 통합 검사의 hand-eye failure는 별도로 남아 있어 전체 workspace green을 주장하지 않는다.

## 이 자료로 판단할 수 있는 범위

이 문서는 기존 검증 기록의 요약이다. 원본 시계열·전체 코드·실행 parameter는 첨부되어 있지 않다. 따라서 프로젝트 상태와 관측된 문제를 설명할 수 있지만 특정 시점의 오차 원인이나 안전 판정의 정확성을 새로 검증할 수는 없다.

상세 분석이 필요해지면 ChatGPT는 목적에 맞는 데이터 내용을 요청한다. 예를 들어 홈 오차 분석에는 같은 시각의 요청 목표, 추정 pose, GT pose와 TF가 필요하고, 반복 정지 분석에는 명령 체인·측정 속도·guard decision·센서 age가 필요하다. 접근할 수 없는 로컬 파일 목록을 사용자에게 먼저 읽으라고 안내하지 않는다.

## 2026-10-08 통합 검사

소스 `3f86ecb`를 통합하기 전 ROS 2 Humble Distrobox에서 monitor core 24개, mission core 51개, BT trace/runtime 설정 24개가 통과했다. 일반 실행에서 ROS opt-in 항목은 각 suite에서 skip되며, monitor의 실제 DDS/QoS·raw 기록·BT 통신 검사는 별도로 활성화해 9개 모두 통과했다.

관련 navigation·monitor 패키지 build/test가 성공했다. colcon 결과는 navigation 160개 오류·실패·skip 0, monitor 33개 중 9개 opt-in skip이며 이 9개는 위 별도 실행으로 확인했다. 웹 production build와 Playwright 8개가 통과했고 실제 실행 모니터 URL이 필요한 웹 2개는 skip됐다. 브라우저 검사는 fixture 기반이며 이번 통합 작업에서 새 Gazebo 주행이나 실물 시험을 수행하지 않았다. 최신 소스의 전체 workspace test 통과를 주장하지 않는다.

## 2026-10-09 통합 검사

소스 `c082d1d`에서 관련 navigation/geometry/grasp/hand-eye Python 검사 157개, mission core 51개, monitor core 24개, scenario 도구 21개가 통과했다. mission·monitor 일반 suite의 ROS opt-in 검사는 각각 9개 skip이며 이번에 별도로 재실행하지 않았다. 웹 build와 fixture 기반 브라우저 검사 9개가 통과하고 live monitor URL이 필요한 2개는 skip됐다. ROS 14개 패키지 build가 성공했다.

원격 SCRUM-426 survey 및 후속 실행은 계속 유지했다. 이번 통합 검사에서 새 Gazebo 왕복·전체 82좌석·실물 시험을 수행하지 않았으며, 관련 과거 결과는 runtime accuracy/scenario 검증 문서의 각 실행 범위로 읽는다. 전체 workspace test 통과를 주장하지 않는다.
