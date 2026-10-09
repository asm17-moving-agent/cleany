# SCRUM-426 Gazebo 시나리오 반복 검증 결과

2026-10-09. 합의한 병렬 한계 측정, 전체 좌석 조사, 대표 fresh/continuous 반복,
이상 상황 반복을 모두 실행했다. **검증 실행 완료이며 모든 경로의 성공을 의미하지 않는다.**
원래 정상 실패와 기대 판정 미통과를 보존했다. 물리 로봇 E2E는 검증하지 않았다.

2026-10-10의 `feat/runtime-integration` 병합과 별도 왕복 smoke 결과는
[통합 확인 기록](runtime-integration-scrum426.md)에 구분해 기록했다.

**50/60 목표 달성:** 노트북 `laptop-command-hold020-continuous60-r2`가 원래 순서
`44 → 16 → 17 → 20 → 09 → 35`를 초기화 없이 10번 반복해 60회를 완료했다.
런타임 SUCCESS는 **60/60**이며, 기록된 Gazebo 실제 위치를 목적지와 복귀점 모두
8cm/0.15rad 이내로 추가 대조한 보수적 집계도 **57/60**이다.

| 좌석 | 런타임 SUCCESS | 실제 위치 기준 충족 |
| --- | --- | --- |
| 44 | 10/10 | 10/10 |
| 16 | 10/10 | 10/10 |
| 17 | 10/10 | 10/10 |
| 20 | 10/10 | 10/10 |
| 09 | 10/10 | 8/10 |
| 35 | 10/10 | 9/10 |

복귀 위치 오차 때문에 추가 집계에서 제외한 것은 17번째(09, 8.29cm),
18번째(35, 8.62cm), 47번째(09, 8.26cm)이다. 원본 런타임 결과는 변경하지 않았다.
목적지 오차 최대 7.39cm, 복귀 오차 최대 8.62cm이며, 이 값은 WORKING/종료 시점의
비동기 GT 표본을 사후 대조한 값이다. 런타임 도착 게이트는 위치추정 값을 사용한다.
물리 로봇 정확도나 접촉 센서 증거로 해석하지 않는다.

GUI 연결 후 58개 미션의 실측 RTF는 **1.607–1.844, 중앙값 1.719**다(요청 RTF 4).
GUI 전 1개와 연결 중 1개는 따로 보존했다. 이번 달성은 정상 왕복 성공률이며
실측 RTF 2 이상을 달성했다는 의미는 아니다.

- 60개 미션 ID·terminal·원본 journal이 모두 일치하고, 한 runtime boot에서 순서와
  시뮬레이션 시계가 연속이다. teleport, 위치추정/장애물 기억 초기화, 취소 주입은 없었다.
- 장면 원문 60개 모두 의자 82개 중 대상 하나만 부재했다. 59번의 전환 요청도 원본
  의자의 형상·위치 및 전후 전체 모델 이름과 일치했다. mesh/material/texture hash
  40개와 시작 로그 로딩 검사를 통과했다.
- 의자 삭제 응답 유실 4건은 실제 반영된 장면을 확인해 중복 실행 없이 처리했다.
  전환 중 장면 조회 7건과 미션 전 조회 1건은 재조회로 복구했다. 요청별 원문·stderr·hash를
  보존했으며 미션 deadline이나 안전/도착 조건은 변경하지 않았다.
- runtime source/config 215개와 검증 도구/profile 31개가 실행 동안 바뀌지 않았다.
  시작 및 후반의 6개 노드 parameter readback도 같았다. 최종 cmd/GT 정지, launch 3개
  exit 0, 자식 30개 정상 종료와 관측기 종료를 확인했다. 요청한 GUI client만 남겼다.
- navigation 167 testcases와 CTest 실행 항목 9개, 검증 도구 52개 검사, 실제 Fortress
  전환 24회 및 시설 44→16 왕복 2회가 앞서 통과했다. 이 검사들을 60회 성공 수에 합산하지 않았다.

최종 근거는 `artifacts/scenarios/first-failure-20261009/`의
`command-hold020-r2-completion-audit.json`, `command-hold020-r2-final-audit.json`,
`command-hold020-r2-transition-final.json`, `command-hold020-r2-precision-final.json`,
`command-hold020-r2-final-provenance-process-audit.json`이다. 원본 실행은
`artifacts/scenarios/laptop-command-hold020-continuous60-r2/worker-00/`에 있다.
동일 소스의 runtime/harness ZIP과 SHA-256도 보존했다. desktop은 오프라인이라
이 최종 후보를 재검증하지 못했으며, 다른 호스트/실행의 성공 수를 합산하지 않았다.

아래는 이전 실패와 개선 과정이다. 최종 결과와 구분해 원본을 유지한다.

**장면 자산 검증 정정:** progress20/center 후보 fixture의 `roly_meshes`가 빈 디렉터리를
가리켰다. 의자 모델 이름과 collision이 남아 있어도 visual 및 센서 장면이 온전함을
증명하지 못한다. 이 후보들의 아래 성공 수는 원본 terminal 집계로만 보존하며, 최종
성공률 증거에서 제외한다. 이전 laptop yaw-calibrated 시험은 원본 로그의 mesh 오류가 0건이며 현재 자산 39개도 존재함을 확인했다. 이 실행은 이미 40/51로 목표 미달이다.
center001 시험은 desktop 23/23, laptop 5/7 terminal 이후 안전 정지했다.
노트북에는 배터리 사용 중 CPU 상한 355.2MHz 저하도 관측됐고 이후 정상 상한으로
돌아왔다. 전원 설정을 변경하지 않았으며 이 기간의 성능을 정상 성능으로 해석하지 않는다.
원본 fixture에서 39개 mesh/material/texture 파일을 검사한 새 실행을 별도로 시작한다.
파일 SHA 검사와 함께 Gazebo 실제 geometry 로딩 오류도 시작 전에 확인한다.
Fortress에서 비 ASCII STL 파일명이 두 표현 모두 로딩에 실패해 해당 STL을 원본과
동일한 바이트의 ASCII 경로로 복사한다. 원본 및 복사본 SHA도 검증한다.
`laptop-center001-assets-continuous60-r3`은 시작 로그 mesh 오류 0건, asset hash
40개(원본 39개와 동일한 STL 복사본 1개)로 재시작했다. sim clock 44.554초에
GUI를 연결했고 의자와 로봇의 실제 렌더링을 화면으로 확인했다. 도구 테스트는 41개
통과했다. 이전 r1은 1건 후 자산 문제로 종료, r2는 시작 검사가 차단해 0건이다.
desktop은 Tailscale 오프라인 상태로 재개하지 못했다. laptop r3은 20건 중
9성공·11실패로 정상 종료해 목표 미달이다. 원본 scene/asset hash와 메시 로딩,
20건의 terminal·journal 일치, 최종 정지/종료를 확인했다. GUI 연결 후 실측 RTF는
1.608–1.777(중앙값 1.675)이며, 이 실패들은 이전 자산 누락과 달리 유효한 장면에서 발생했다.
첫 실패는 두 번째 20번 복귀였다. 잠긴 실제 지도/자세의 C++ 재생은 직전 529개 중
87개 허용, 실패 시 529개 전부 거부를 그대로 재현했다. 직전 발행 명령을 그대로
유지하면 0.125초에 기존 격자 충돌 검사가 거부하지만, 계속 가속하는 원래 예측 경로는
통과한다. 실제 명령 유지 시간이나 지연의 원인을 이 재생만으로 확정하지 않는다.
다음 `cad_fortress_command_hold.yaml` 후보는 기존 조건에 첫 명령 0.2초 유지 검사를
추가한다. 이전 검사/점수/안전·도착·시간 기준은 유지하며, 결과는 새로운 60회로 검증한다.
후보 build 및 167 navigation testcases가 통과했다(CTest 실행 항목 9개를 더한 colcon
집계는 176). `laptop-command-hold020-continuous60-r1`은 원래 순서의 **49회 모두
SUCCESS**였으나 50번째 미션 전 의자 복원 RPC timeout으로 종료됐다. Gazebo 로그에
응답 전송 `Host unreachable`이 있으며, 당시 생성이 적용됐는지는 확인할 장면이 없어
미확정이다. 49건의 원본 terminal·journal이 일치하고 각 미션은 정지했다. 마지막 trace도
IDLE·cmd/GT 0이지만 예외로 `result.final_latest`가 누락됐으므로 이를 소급 보충하지 않는다.
GUI 연결 후 RTF 1.667–1.881, 중앙값 1.774이며 **60회 미완료로 목표 미달**이다.
새 전환 도구는 실제 장면을 읽어 응답 유실과 미적용을 구분하고, 미적용 요청만 최대
세 번 재시도한다. 예상 외 모델 변경과 장면 읽기 실패는 계속 거부한다. 52개 도구 검사와
분리된 실제 Fortress 24회 전환(생성/삭제 응답 유실 및 미발송 요청 주입 포함)을 통과했다.
49회 실행 종료에서 launch와 자식에 중복 SIGINT가 전달된 흔적도 있어, graceful 종료는
launch에만 한 번 전달하고 기존 18초 후 강제 종료 경계는 유지한다. 이 변경과 재검증은
주행·안전 기준을 완화하지 않는다. 시설 월드 44→16 왕복 2/2, 의자 전환·정지·DB 대조가
통과했고 자식 30개 모두 exit 0이었다. 종료 중 ZipFile destructor 경고는 별도 보존했다.
최종 obstacle snapshot의 CRC 및 실제 runtime loader 복원은 통과해 파일 손상은 없었다.
기존 49회와 합산하지 않고 새 `laptop-command-hold020-continuous60-r2`에서 원래
60회를 다시 검증해 위 최종 결과를 얻었다. 실제 parameter readback에서
유지 검사 두 값과 실행별 파일 경로 외 기존 설정 차이가 없음을 확인했다.
노트북 GUI는 sim clock 131.807초에 새 partition으로 연결했고 실제 장면을 확인했다.
아래 표는 개선 전 결과이며 목표 달성을 증명하지 않는다. 첫 실패에서 진단·수정 후
재실행한다. 기존 실패 자료, 좌석 순서, 대상 의자만 제거하는 조건, 도착·안전·시간
기준을 유지한다. 2026-10-09 추가 진단에서 두 독립 20번 실행의 wheel yaw gain
1.0465/1.0469를 측정했다. `yaw_scale: 1.047` 보정 후 20번·17번 단독 왕복은
각각 SUCCESS와 정지/종료 검사를 통과했다. 이후 원래 순서의 6좌석 연속 재검증도
6/6 SUCCESS였고, 82개 의자 중 대상만 제거한 각 scene 및 정지/종료 검사가 통과했다.
20번 단독 실측 RTF 2.024, GT 목적지/복귀 오차 2.6/5.9cm이다. 노트북의 본 60회
시험 `laptop-yaw-calibrated-continuous60-r1`은 51건 중 40성공·11실패로 종료됐다.
데스크탑도 20번 단독 SUCCESS(RTF 2.150), navigation/odometry 190 tests 통과 후
6좌석 연속 6/6 SUCCESS와 원본 의자 scene 검사를 통과했다. 이후
`desktop-yaw-calibrated-continuous60-r1`은 45건 중 34성공·11실패로 종료됐다.
실패가 11건이면 남은 시도가 모두 성공해도 50/60에 미달하므로 시험을 종료한다.
두 실행 모두 목표 미달이다. 각 terminal과 저장된 journal 51/45건이 일치하고,
정지·launch 종료 및 입력/odom source/navigation binary hash 검사를 통과했다.
호스트별·실행별 결과를 합산해 60회 달성으로 처리하지 않는다.
원본은 `artifacts/scenarios/first-failure-20261009/` 및 `laptop-yaw-calibrated-*`에 보존했다.

노트북 본 시험은 사용자 요청으로 sim clock 2115.311초에 같은 partition의
Gazebo GUI client를 연결했다. 서버·미션은 재시작하지 않았다. 연결 시점과 명령은
`first-failure-20261009/gui-continuous60-r1/attach.json`에 기록했고, 최종 RTF는
headless / GUI 연결 중 / GUI 연결 후 미션으로 구분한다. GUI 전후 수치를
동일한 부하의 성능 비교로 해석하지 않는다.

본 시험 중 노트북 20번째(16번 outbound), desktop 16·22번째(20번 return)에
진행 실패를 기록했다. 실패 후 다음 미션도 같은 scene에서 실행하며 실패를 제외하지
않는다. desktop 두 번째 실패의 추가 관측에서는 Nav2가 새 0속도를 계속 발행하고,
collision monitor의 0속도 발행 종료 뒤 최종 guard가 COMMAND_TIMEOUT으로 정지했다.
같은 시점의 DWB 평가에서 정지 25.4796, 가장 낮은 이동 후보의 누적 점수 26.8447을
확인했다. 경로/방향 전환 점수가 정지를 선호하는 사례이며, 이동 후보의 점수는
short-circuit될 수 있으므로 모든 후보의 완전한 검증이나 atomic replay로 주장하지 않는다.
추가 명령 관측은 읽기 전용이며 별도 실행 부하를 발생시킨다. 원본 CDR와 후속 costmap
응답은 `first-failure-20261009/desktop-continuous60-r1-second-seat20-stall/`에 보존했다.

다음 후보는 동일한 보정·안전 설정에서 fixture의 `FollowPath.PathProgress.scale`만
1에서 20으로 변경했다. 미터 단위 진행 점수와 격자 단위 거리 점수의 차이로 정지가
선택되는 사례를 대상으로 한다. 실제 ROS parameter readback에서 해당 값과 실행별
출력 경로 외 차이가 없음을 확인했다. 이는 시험 후보이며 source 기본값은 아직 1이다.
`desktop-yaw-progress20-continuous60-r1`은 20건 중 9성공·11실패로 목표 미달이다.
10번째(20번 outbound)에 모든 DWB 후보가 ObstacleFootprint로 거부됐고 이후 미션도
같은 위치에서 출발하지 못했다. 저장된 journal 20건, 실제 의자 scene 20개와 정지/종료
기록을 보존했다. `laptop-yaw-progress20-continuous60-r1`도 34건 중 23성공·11실패로
목표 미달이다. 24번째(35번 outbound)의 모든 후보가 ObstacleFootprint에서 거부된 뒤
후속 미션도 출발하지 못했다. 34건의 journal 일치, 정지와 종료 검사는 통과했다.
노트북 GUI는 새 partition으로 재연결했고 sim clock 151.014초의 연결 기록과 명령은
`first-failure-20261009/gui-progress20-continuous60-r1/attach.json`에 보존했다.
추가 명령 관측은 정체 시에만 DWB 평가를 구독하도록 변경했으므로 이전 시험과 RTF를
동일한 관측 부하에서 측정한 것으로 해석하지 않는다.

desktop progress20의 실패 지도에서는 정지한 footprint와 lethal cell 하나가 겹쳤다.
이후 중앙 장애물 비용(`BaseObstacleCritic`, scale 0.2)을 추가한 후보는 목표 직전
정지를 유발해 7건 모두 실패했다. 기록된 16번 평가에서 중앙 비용은 정지 38×0.2,
도착 쪽 이동 70×0.2였다. 실패 후보는 7번째 terminal IDLE와 실제 정지를 확인한 뒤
종료했다. `candidate-stop-request.json`, 원본 trace, 일치하는 journal 7건을 보존했다.
이 실행은 의도적 조기 종료(KeyboardInterrupt)이고 완료된 60회 시험으로 집계하지 않는다.
다음 후보 `desktop-yaw-progress20-center001-continuous60-r1`은 중앙 비용 scale만
0.01로 낮춰 원래 60회 순서로 시작했다. 기존 충돌·도착·시간 기준은 유지한다.
후보 점수의 오프라인 민감도 계산은 실행 성공이나 모든 critic의 완전한 재생 증거가 아니다.
노트북도 `laptop-yaw-progress20-center001-continuous60-r1`을 독립 실행했으나 위 자산 문제로 종료했다.
두 호스트의 후보 `nav2_safety.yaml` SHA-256은 동일하다. GUI는 새 노트북 partition으로
다시 연결했으며 sim clock 110.361초의 연결 기록을 `gui-progress20-center001-continuous60-r1/`
아래에 보존했다. 노트북의 이전 명령 관측에서는 debug topic의 IDLE 값 때문에 정체
캡처가 시작되지 않았으므로, 새 관측기는 실제 worker snapshot service 결과를 읽는다.
관측은 읽기 전용이며 이전에 없던 DWB 캡처를 소급해서 주장하지 않는다.
현재 후보는 `tools/scenario_validation/profiles/cad_fortress_accuracy.yaml`과
`prepare_profile.py`로 재현할 수 있다. 생성된 fixture의 설정·지도·월드·목표 파일
10개가 해당 노트북 후보와 byte 단위로 같았다. 이 비교는 메시를 포함하지 않아
장면 전체의 동일성을 증명하지 못했다. 이후 메시와 텍스처의 존재 및 SHA 검사를 추가했다.
도구 테스트 통과는 Gazebo 성공률과 별개다.

## 실행 환경과 의자 조건

- 별도 branch/worktree `test/scrum-426-scenarios`; local root
  `/home/ehdrms/workspace/cleany/scrum-426-scenarios`, desktop root
  `/home/donglee/workspace/cleany/scrum-426-scenarios`.
- desktop: Ryzen 5 9600X / RTX 5070, Ubuntu 22.04 ROS Humble/Fortress container,
  NVIDIA Ogre2 EGL. laptop: aarch64 `a16-fedora`, freedreno/Adreno X2-90 Ogre2.
- 각 worker의 ROS domain, Gazebo partition, DB, 결과/로그를 분리했다.
  요청 RTF 4, physics step 0.0066667초, compact static scene, reactive safety.
- 82개 좌석과 82개 의자 대응 검사. 원래 누락된 43/13 의자를 복원한 뒤
  **대상 의자 하나만 제거**한다. 모든 다른 모델을 보존한 82개 월드 검사 통과.
- continuous는 이전 의자를 복원하고 다음 의자를 제거한다. 실제 scene/info 원문
  60개를 독립적으로 파싱하고 hash를 대조했다. 모두 81개 의자, 해당 대상 하나만 부재.
- navigation/sensors는 Gazebo 실측, perception/planning/manipulation은 mock.
  팔 제외 simulation profile이며 실제 로봇·팔·물리 e-stop·D435 분리 증거가 아니다.

## 데스크탑 병렬 한계와 처리량

RTF는 미션 `/clock` 증가량 / wall time이다. 처리량은 준비/종료를 포함한 batch
시간 기준 성공 왕복/시간이다. 두 값을 합하거나 같은 지표로 해석하지 않는다.

| worker 수 | 긴 13번 왕복의 실측 RTF | 검증 결과 |
| --- | --- | --- |
| 1 | 2.145 | SUCCESS; 짧은 43번은 2.153 |
| 2 | 1.636–1.709 | 긴 왕복 3회 측정 모두 SUCCESS |
| 3 | 1.192–1.298 | 긴 왕복 3회 측정 모두 SUCCESS |
| 4 | 0.782–0.814 | 3회 × 4개 = 12/12 SUCCESS |
| 5 | 0.484–0.513 | 3회 × 5개 = 15/15 TIMEOUT |
| 6 | 0.335–0.343 | 6/6 TIMEOUT |

현재 profile/deadline과 13/43 기준 경로에서는 **왕복 성공 한계 4개**, 실측 2×는
단일 worker에서 확인됐다. 전체 좌석의 성공 한도나 하드웨어의 일반 한도로 주장하지 않는다.
5개에서는 짧은 43번이 성공해도 긴 13번이 실패하므로 짧은 시험만으로 한도를 정하지 않았다.

긴 왕복의 2/3-worker 처리량 3회 중앙값은 63.65/73.25 성공 왕복/시간이다.
전체 좌석 조사에는 3-worker를 선택했다. 단일 worker는 38.66이었다.
4/5-worker CPU 평균은 약 89–92%/96–98%, GPU 최대는 16% 이내,
5-worker MemAvailable은 약 31 GiB 이상이었다. CPU 포화와 속도 저하가 함께 나타났지만
독립 profiler로 각 계산·대역폭 병목을 분리한 결과는 아니다.

첫 2-worker configure 응답 누락 사례는 중단 로그로 보존했다. harness는 discovery 후
명시적인 lifecycle STARTUP을 사용한다. 이 준비 문제를 미션 성공으로 집계하지 않았다.

## 전체 좌석과 대표 반복

| 범위 | 실행 수 | 결과 |
| --- | --- | --- |
| baseline 전체 좌석 조사 | 82 | 27 SUCCESS / 55 FAILED |
| baseline 실패 55개 수정 후 조사 | 55 | 17/22 두 좌석 추가 SUCCESS; 조사 단계 latest 29/82 |
| 대표 6개 fresh × 10 | 60 | 44 SUCCESS / 16 FAILED |
| 한 인스턴스 6-seat sequence × 10 | 60 | 5 SUCCESS / 55 FAILED |

추가로 01 단독 및 13/43 대조를 수행해 correction queue는 총 58 missions다.
대표는 조사 단계에서 측정한 near/far/narrow mapped path/large turn/arrival error/
guard intervention 역할의 44/16/17/20/09/35다. 반복 실패 때문에 대표를 교체하지 않았다.

| 좌석 | fresh SUCCESS / 10 | continuous SUCCESS / 10 |
| --- | --- | --- |
| 44 | 10 | 2 |
| 16 | 10 | 2 |
| 17 | 2 | 1 |
| 20 | 2 | 0 |
| 09 | 10 | 0 |
| 35 | 10 | 0 |

continuous 첫 순환은 44/16만 성공, 두 번째는 44/16/17 성공, 3–10번째는 모두 실패했다.
실패 후 현재 위치·localization·obstacle memory를 유지하고 다음 미션을 실행했다.
teleport, 지도/기억 초기화, 안전 기준 완화로 통과시키지 않았다. fresh와 continuous는
worker 수와 scene 유지 방식도 달라 직접적인 성능 인과 비교로 해석하지 않는다.

전체 82개, correction 58개, fresh 60개와 continuous 60개의 실제 terminal mission ID와
좌석을 각각 확인했다. 모든 실행의 최종 cmd/GT 정지 및 launch 종료 검사도 통과했다.
continuous의 기대 정상 SUCCESS 판정은 **미통과**이며 실행 완료와 구분한다.

## 이상 상황 검증

두 호스트에서 7종 smoke를 각각 수행했고 각각 기대 판정 6/7이 통과했다.
노트북의 독립 fresh 5회 반복은 입력 35개와 terminal status 35개가 정확히 일치한다.

| 조건 | 반복 | 기대 판정 통과 |
| --- | --- | --- |
| outbound cancel | 5 | 5 |
| return cancel | 5 | 5 |
| synthetic safety stop | 5 | 5 |
| depth-clearance adapter loss | 5 | 0 |
| inserted obstacle | 5 | 5 |
| planner process loss | 5 | 5 |
| blocked target | 5 | 5 |

총 30/35 통과. 센서 안전 입력 유실 5회 모두 STALE_DEPTH와 실제 정지
(0.404–0.419 wall seconds)를 관찰했지만 cleanup cancel 후 return 실패로 terminal
FAILED였다. 기대 CANCELLED 판정은 통과하지 않았고 기대값을 변경하지 않았다.
raw depth camera는 계속 동작했으므로 물리 센서 분리 시험이 아니다.
장애물 clearance는 sampled GT/configured padded XY body 거리이며 접촉 센서 증거가 아니다.

## 실패 분석·개선과 검증 한계

- endpoint 회전의 한 구간 conservative bound가 과도하게 넓어 안전한 half-turn을
  거부했다. 회전을 작은 연속 검사 구간으로 나누고 같은 full-cell SAT/arc 안전 판정을
  유지했다(`3d6604d`). 저장 01 지도는 경로를 반환했지만 실제 01 왕복은 실패했다.
- 고정 body heading으로 통과하지 못하는 room 경로에 32방향 sparse body-heading
  탐색을 추가했다(`5d5c01b`). 기존 공유 2초 예산 안에서 full footprint/edge/live 검사한다.
  저장 지도에서는 성공했으나 실제 Gazebo controller progress 실패는 남았다.
- DWB 정지와 전진의 기존 점수가 같던 수집 사례를 확인하고 연속 path progress
  critic을 추가했다(`cabd0ea`). 기존 장애물·속도·도착·freshness/timeout guard는 유지했다.
  reactive mode에서는 pan gate가 observation_only이므로 이전 혼합 명령 차단 설명은
  이 모드의 원인 근거가 아니다. 수집 snapshot은 atomic controller replay가 아니다.
- 독립 laptop v5 overlay: **175 tests, 0 errors/failures/skipped**, plugin package build 완료.
  13/43 두 왕복 SUCCESS, 실측 RTF 1.883/1.701, 정지와 종료 검사 통과.
  room-a4-01/17/20 3개 병렬은 모두 FAILED(RTF 1.320/1.325/1.347).
  20은 target navigation OK와 mock 작업 후 return 실패였다. 전체 문제 해결로 주장하지 않는다.
- v5 20번 return 저장 지도는 start/goal footprint가 각각 안전했지만 32방향 탐색이
  129,456 states를 0.473초에 소진해 연결되지 않았다. 이는 이산 탐색 실패이며
  연속 공간의 실제 통행 불가능성 증명은 아니다.
- continuous 후반 44번 실패 snapshot을 size/FNV 검증 후 별도 보존했다.
  start footprint는 안전했지만 goal footprint는 거부됐다. search/live 지도의 lethal
  셀 6개가 goal footprint와 겹쳤으며 원래 정적 PGM에서는 해당 영역이 free였다.
  같은 지도의 32방향 탐색도 2,618,046 states/13.033초를 소진해 실패했다(timeout=false).
  이 진단 예산을 runtime에 적용하지 않았다. 해당 후반 관찰의 GT/map pose 차이는
  약 0.25m였지만 이것만으로 lethal 셀의 센서/TF 원인이나 실제 충돌을 확정하지 않는다.

8cm runtime arrival, 180초 navigation timeout, freshness, 속도와 정지 기준을 유지했다.
정상 왕복의 17/20 불안정, 지속 scene의 관측 지도/위치추정 문제와 depth 입력 유실 시
취소·복귀 terminal 정책은 후속 개선 항목이다. 모두 성공으로 완료 처리하지 않았다.

## 원본 증거와 종료 감사

`artifacts/scenarios/`에 fixture, 입력/월드/tool/binary hash, trace, result, shutdown,
resource samples, scenario queue/status, 실제 scene/info 원문과 실패 map bytes를 보존했다.
호스트/architecture/install prefix와 binary hash로 desktop v2와 laptop v5를 구분한다.
baseline 구현 `076b893`, desktop 반복 source `f9909ca`, local navigation 최종 변경
`cabd0ea`이다. 기존 branch/worktree의 사용자 변경은 건드리지 않았다.

- `plan_completion_audit.json`: scope별 queue와 미션 판정.
- `final_continuous_verdict.json`, `final_chair_scene_audit.json`: 60회 결과와 원문 의자 검증.
- `final_remote_manifest.json`, `final_sync_audit.json`: desktop 원본 5,723개,
  2,555,038,639 bytes를 local에 복사하고 전 파일 SHA-256 일치 확인.
- `final_remote_process_audit.json`, `final_local_process_audit.json`: desktop/local
  자식 기록 816/180개 확인, 이 작업의 남은 launch/worker/observer 없음.
- `planner-replay-continuous-late/`: 후반 실패의 immutable map과 별도 geometry/cell/SE2 재생.
- `laptop_repeat_verdicts.json`, `capacity_report.json`, 각 queue의 `*.status.json`.

Jira에 메시지를 보내거나 상태를 변경하지 않았다. 문서/로컬 commit과 원본 증거를 남겼다.
과정·중간 실패·추가 진단의 상세 기록은 [단계별 조사 기록](scrum-426-scenarios-history.md),
실행 도구와 mode 구분은 [검증 도구 README](../../tools/scenario_validation/README.md)를 따른다.

## 재실행

같은 source/version과 tested install prefix를 선택하고 새 이름/domain을 사용한다.
실행 중인 install을 덮어 빌드하지 않는다. fixture는 기존 검증 입력을 사용한다.
최종 60회 후보의 노트북 재실행 예(출력 이름/domain은 미사용 값을 선택):

```bash
python3 tools/scenario_validation/prepare_profile.py \
  artifacts/scenarios/fixture artifacts/scenarios/command-hold-recheck-fixture \
  --profile tools/scenario_validation/profiles/cad_fortress_command_hold.yaml
python3 tools/scenario_validation/parallel.py \
  --root "$PWD" --container ros2-humble --count 1 \
  --name laptop-command-hold-recheck --domain-base 204 \
  --seat seat-44 --sequence seat-44,seat-16,seat-17,seat-20,seat-09,seat-35 \
  --repeats 10 --failure-limit 11 \
  --fixture "$PWD/artifacts/scenarios/command-hold-recheck-fixture" \
  --runtime-overlay "$PWD/ros2_ws/install-scenario-runtime/setup.bash" \
  --navigation-overlay "$PWD/ros2_ws/install-navigation-v7/setup.bash"
```

노트북 v5 대조 왕복 재실행 예:

```bash
python3 tools/scenario_validation/parallel.py \
  --root "$PWD" --container ros2-humble --count 2 \
  --seats seat-13,seat-43 --name laptop-reference-recheck \
  --runtime-overlay "$PWD/ros2_ws/install-scenario-runtime/setup.bash" \
  --navigation-overlay "$PWD/ros2_ws/install-navigation-v5/setup.bash" \
  --domain-base 160
```

대표 fresh 반복은 `representative_fresh_jobs.json`, continuous 순서는
`representative_continuous_jobs.json`, 이상 상황 35개 입력은
`laptop_scenario_repeat_jobs.json`에 있다. 각 job에 새 고유 이름을 부여해
`queue.py --root "$PWD" --jobs INPUT.json`으로 재실행한다. 기존 결과 디렉터리는 덮어쓰지 않는다.
