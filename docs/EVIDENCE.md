# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

## 2026-10-23 기하학적 후보 생성으로 운영 전환

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble과 commit
`3950218d70cbebd4ce3278e7602bc276482c0471`의 미커밋 작업 내용에서 검사했다.
공통 collection runtime의 기본 후보 방식을 `geometric`으로 변경하여
`make manipulation-run`과 dev-monitor에 적용했다. cuRobo IK/MoveIt 경로와
20mm 추가 접근 보정 및 8cm, 6cm, 4cm 준비 위치는 유지했다.

`make test-jetson-tools JETSON_PYTEST_ARGS="-k test_collection_runtime"`는
103개 통과, 3개 제외였다. shell 기본값, 기존 검사와 실행 문서만 변경하여
ROS 패키지 재빌드는 필요하지 않았다. AnyGrasp container를 종료하고 native
기하학적 후보 서버가 유일한 `/grasp/plan` 제공자인 것을 확인했다.

전환 중 Docker 네트워크 재생성과 함께 DDS 전송 오류 및 feedback timeout
보호 정지가 발생해 첫 readiness 검사가 실패했다. 기존 복구 명령으로 powered
hold를 인계했고 goal, RAM, torque 쓰기 없이 복구됐다는 하드웨어 로그를 확인했다.
유휴 Action 서버의 남은 fault 상태도 재시작으로 제거했다. 최종 검증에서 양팔과
그리퍼 controller가 active, guard가 ready이며 실행기 fault가 없음을 확인했다.

06:58:58의 새 카메라에서 파란 물체(object 2, 검출명 power bank)를 선택했다.
target 173점과 context 3,760점으로 후보 53개를 약 0.912초에 생성했다.
실제 팔 이동 명령은 보내지 않았으며 경로 계획과 실물 파지 성공은 검증하지 않았다.

원본은 `../cleany-data/reports/geometric-switch-20261023/`의
`deployment.json`, `verification.json`, `implementation.patch`, `runtime-tests.xml`,
`deployment.log`, `control-recovery-held-lock.log`, `action-restart.log`에 있다.
카메라와 점군은 `../cleany-data/reports/geometric-camera-20261023-065858/`에 보존했다.

## 2026-10-23 승인 대상만 비교하는 재검증

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble과 commit
`3950218d70cbebd4ce3278e7602bc276482c0471`의 미커밋 작업 내용에서 검사했다.
승인 대상이 유지되는데 다른 검출의 추가, 소실, 위치나 분류 변화로 승인이 거절되던
전체 장면 비교를 제거했다. 승인 대상의 추적, 위치, bbox, 분류, 깊이, 크기와 목적지
검사 및 승인 이후 실행기의 경로와 충돌 검사는 유지했다.

추가한 주변 검출 변동 검사 7개는 수정 전에 실패했고, 수정 후 관련 검사 22개가
통과했다. 전체 `make test-dev-monitor`는 182개 통과, opt-in 검사 18개 제외였다.
별도 domain 231의 합성 ROS 검사 1개에서 주변 검출 소실 후 승인, 이동한 대상 거절,
원래 승인 Goal의 단일 전송을 확인했다. 과거 실패 3건의 원래 제안과 새 관측을 당시
시각으로 재생하여 모두 통과했으며, 승인 대상을 변경하지 않았다.

웹과 ROS 패키지 4개 빌드를 완료했다. Vite의 기존 `use client`와 bundle 크기 경고가
있었으며 종료 코드는 0이었다. 유휴 상태에서 Dev Monitor만 재시작하고 변경된 코드와
화면 자산의 제공, 제어 backend 준비 상태를 확인했다. 실제 로봇 Action은 보내지
않았으며 실물 파지 검증은 수행하지 않았다.

원본은 `runtime/scene-revalidation-diagnosis-20261023/`의 작업 UUID별 JSON,
`provenance.json`, `target-only-replay.json`, `monitor-regression.xml`,
`ros-approval-regression.xml`, `build.log`, `deployment.json`에 보관한다.

## 2026-10-23 수거 파지 목표 추가 삽입 20mm

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble과 commit
`3950218d70cbebd4ce3278e7602bc276482c0471`의 미커밋 작업 내용에서 검사했다.
공용 수거 설정의 `grasp_approach_offset_m`을 0.020m로 적용했다. TCP 정의와 닫힘
방향 보정은 유지하며, 원래 8cm, 6cm, 4cm 준비 위치에서 보정 목표까지의 거리는
10cm, 8cm, 6cm다. 집중 검사 11개와 관련 회귀 검사 131개가 통과했고 native
1개 패키지와 motion container 5개 패키지 빌드가 완료됐다. 기존 NumPy/SciPy
버전 경고와 container의 setuptools_scm 경고가 있었으며 종료 코드는 0이었다.

planning support만 복구하여 real_control PID 55052를 유지했고 운영 파라미터
0.02를 확인했다. 05:34:33 카메라의 정면 캔, snapshot
`rgbd-1792701273610493164-037980`에서 target 700점과 context 7680점으로
AnyGrasp 후보 58개를 생성했다. 왼팔은 후보 5에서 두 경로 계획을 통과했고,
4cm 준비 위치와 보정 목표 사이의 거리가 60mm인 것을 실제 IK 요청에서 확인했다.
오른팔은 상위 32개 검사에서 실패했다. 실제 팔과 그리퍼 이동 명령은 보내지
않았으므로 이 결과는 실물 파지 성공이나 삽입량 캘리브레이션의 근거가 아니다.

원본은 `../cleany-data/reports/grasp-insertion-20mm-20261023/`의
`verification.json`, `implementation.patch`, `regression-tests.xml`,
`selected-ik-diagnostics.json`, `deployment.json`에 보존했다. 카메라, 점군과
Action 결과는 `../cleany-data/reports/grasp-insertion-20mm-camera-20261023-053433/`에 있다.

## 2026-10-23 캔 크기 추정 변동의 원인 분리

Jetson Orin NX, ROS 2 Humble과 commit
`3950218d70cbebd4ce3278e7602bc276482c0471`의 미커밋 작업 내용에서 승인 작업
`23d7ed83-a7c3-41a8-b4c3-03a09d659be6`의 크기 필터 탈락을 조사했다. 당시 Action
결과를 읽어 같은 캔의 짧은 바닥면 추정값 9.297cm와 13.368cm를 확보했으며,
11cm 제한에 의한 탈락을 재현했다. 당시 RGB-D 원본과 마스크는 120초 캐시가
만료되어 확보하지 못했다.

현재 장면에서 약 10.07초 동안 20개 RGB-D 입력을 수동 구독으로 수집하고 운영
TensorRT 모델과 같은 geometry 함수로 분석했다. 운영 로봇 Action과 Gemini 호출은
보내지 않았다. 짧은 바닥면 추정값은 5.388~20.137cm였고 20개 중 6개가 11cm를
넘었다. 첫 관측의 마스크와 평면을 고정하고 깊이만 바꿔도 5.371~20.322cm로
변동했다. 마스크 IoU는 첫 관측 대비 0.9942 이상, 평면 법선 변화는 0.224도 이하였다.

물체로 채택된 약 11,300개 점 중 캔 위쪽 경계의 0~15개 점은 카메라 깊이
70.4~73.7cm, 추정 평면 위 높이 5.02~8.52mm였다. 이 점들은 5mm 물체 높이
조건을 통과하여 PCA와 min/max 상자 범위에 포함됐다. 진단용 0.6m 깊이 분리로
이 점들만 제외하면 폭은 5.268~5.568cm였다. 이는 새 관측의 원인 분리 실험이며,
과거 두 프레임의 직접 재생 또는 운영 필터 변경이 아니다.

수집 도구의 최초 설정 일괄 조회는 미초기화 optional parameter 때문에 실패했고,
개별 조회로 수정한 뒤 수집, 분석과 그래프 생성을 완료했다. 운영 perception 코드와
11cm 제한은 유지했다. 원본은 `runtime/footprint-diagnosis-20261023/`의
`captures.npz`, `masks.npz`, `parameters.json`, `analysis.json`, `diagnostics.json`,
`diagnostic.png`, `provenance.json`과 `historical/`에 보관한다. 재실행 절차는
[실물 크기 추정 변동 진단](TESTING.md#실물-크기-추정-변동-진단)을 따른다.

## 2026-10-23 Dev Monitor 작업 제안과 사용자 승인

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, Python 3.10.12와 commit
`3950218d70cbebd4ce3278e7602bc276482c0471`의 미커밋 작업 내용에서 검사했다.
Mission Manager를 실행하지 않고 규칙 기반 대상 제안, 승인된 snapshot/object의 단일
Action, 결과 이후 새 제안의 승인 대기를 연결했다.

`make test-dev-monitor`는 174개 통과, opt-in 검사 18개 제외였다. Domain 231의 별도
ROS fixture는 실제 InspectScene, snapshot 조회와 기존 Action CLI를 연결하여 관측
이미지 일치, 이동한 대상 거절, 정확한 승인 Goal 1회 전송, 다음 제안과 건너뛰기의
비실행을 확인했다. 이는 합성 Action 결과를 사용한 계약 검사다. Playwright의
`operator.spec.ts` 9개와 웹 빌드, ROS 패키지 4개 빌드가 통과했다. 데스크톱과 모바일
제안 화면을 캡처해 확인했다.

운영 monitor를 갱신한 뒤 실제 인식만 요청한 작업
`1d10ae5b-a2b2-4088-b21f-02dc2fd417d4`가 약 7.49초에 완료됐다. 캔을 물체 ID 1로
추천하고 관측 JPEG와 함께 `READY`로 반환했다. 최초 요청은 별도 계획 검증 프로세스가
실행 잠금을 보유하여 HTTP 409로 차단됐고, 해당 프로세스 종료 후 요청했다. 실물
승인 또는 파지 Action은 보내지 않았으며 이 결과를 실제 파지 성공으로 해석하지 않는다.

원본은 `../cleany-data/reports/dev-monitor-proposals-20261023/`의
`live-proposal-job.json`, `live-proposal.jpg`, `ros-proposal-validation.xml`,
`proposal-desktop.png`, `proposal-mobile.png`, `verification.json`에 있다.

## 2026-10-23 접근 준비 자세의 4cm 대체 검사

Jetson Orin, ROS 2 Humble과 commit `3950218d70cbebd4ce3278e7602bc276482c0471`의
미커밋 작업 내용에서 수거 준비 자세를 8cm, 6cm, 4cm 순서로 확장했다.
`pregrasp_fallback_offsets_m=[0.06, 0.04]`를 사용하며, 앞선 준비 자세의 IK가
없거나 기존 허용 오차를 넘을 때만 다음 거리를 검사한다. 통과하면 뒤의 거리는
생략한다. 기존 단일 거리 설정은 목록이 비어 있을 때 계속 사용할 수 있다.

루트 Makefile의 집중 검사 30개와 관련 회귀 197개가 통과했다. 순서, 조기 중단,
4cm의 충돌 거부, 취소와 실제 선택한 준비 자세의 접근 통로 검사를 포함한다.
native 패키지 1개와 motion container 패키지 5개 빌드를 완료하고 계획 지원만
재시작했다. real_control PID와 운영 인식 container는 유지했다.

04:44 KST의 새 캔 입력에서는 후보 66개를 생성했고 팔별 상위 32개까지 검사했다.
왼팔 후보 2, 9, 17에서 실제 8cm, 6cm, 4cm 호출을 확인했다. 4cm에서도 가장 가까운
접근 방향 오차가 각각 약 32.1도, 32.1도, 30.9도로 허용치 30도를 넘었다.
새 입력은 양팔 모두 실패했고 경로 탐색에 진입하지 않았다. 이전 성공 입력인
저장 후보 71개를 같은 운영 서버에서 재검사하면 후보 7의 6cm 준비 자세와 두 구간
경로가 계속 통과했다. 거리 검사 확장을 모든 후보의 계획 성공으로 해석하지 않는다.
실제 팔과 그리퍼 이동 명령은 보내지 않았다.

원본은 `jetson-orin:/home/cleany/cleany-data/reports/pregrasp-4cm-20261023/`과
`pregrasp-4cm-camera-20261023-044405/`에 보관한다. `verification.json`은 실제 결과,
`implementation.patch`와 `change-manifest.json`은 이번 변경, XML과 빌드 로그는
검사 결과, `deployment.json`은 제어 프로세스 유지, `fresh-ik-stats-*.json`과
카메라 폴더의 CDR 및 피드백은 거리별 탈락 근거다. 별도 8→4cm 임시 진단 노드는
action 호출 없이 종료했으며 실제 거리 검증은 운영 서버의 8→6→4cm 경로에서 수행했다.

## 2026-10-23 캔의 팔 선택 및 경로 계획 실패 수정

Jetson Orin, ROS 2 Humble과 commit `3950218d70cbebd4ce3278e7602bc276482c0471`의
미커밋 작업 내용으로 운영 AnyGrasp 및 `curobo_ik_moveit` 경로를 검사했다.
04:17 KST에 새로 촬영한 캔에서 후보 71개를 생성하고 팔별 상위 32개까지 검사했다.
기존 설정은 양팔 모두 실패했다. 왼팔 후보 7의 파지 자세는 통과했으나 0.08m
준비 자세의 가장 가까운 IK 방향 오차가 약 33.5도로 허용치 30도를 넘었다.
양팔 모두 경로 탐색 호출은 없었다. 이 유한 탐색 결과가 해당 자세의 기하학적
불가능을 증명하지는 않는다.

같은 후보에서 준비 거리를 0.06m로 바꾸면 왼팔의 두 구간 경로가 통과했다.
수거 프로필에 `pregrasp_fallback_offset_m=0.06`을 추가해 0.08m 준비 자세 IK가
없거나 허용 오차를 넘을 때만 한 번 대체 검사하도록 수정했다. 후보와 파지 자세,
기존 위치 및 방향 허용치, 전체 장면 충돌 검사와 경로 검증은 유지했다.
방향 보정 검색만 추가한 별도 실험은 양팔 모두 실패했으며 그 변경은 적용하지 않았다.

루트 Makefile로 집중 회귀 12개와 관련 전체 회귀 179개가 통과했다.
native 패키지 1개 및 motion container 패키지 5개 빌드를 완료했다.
확대 회귀의 첫 실행에서 기존 예열 테스트가 `continue_work` 인자를 반영하지 않아
실패했으며, foreground 요청 시 양보하는 조건까지 검사하도록 기대값을 갱신했다.
로컬 시각화 venv에는 pytest가 없어 자동 검사는 Jetson native 환경에서 실행했다.

`collection-recover-planning`으로 계획 지원만 재시작하고 real_control PID를 유지했다.
운영 서버에서 동일한 후보의 왼팔 계획과 반복 계획이 통과했다. 같은 요청 안에서
0.08m 준비 자세는 약 31.5도로 탈락하고 0.06m 준비 자세는 약 27.7도로 통과했다.
04:26 KST의 새 촬영에서는 후보 49개 중 왼팔 후보 0을 선택했으며, 준비 자세 방향
오차 약 29.1도로 두 구간의 경로가 통과했다. 오른팔은 이 장면에서 계속 IK에
실패했다. 모든 검사는 plan-only로 실행했고 팔과 그리퍼에 이동 명령을 보내지 않았다.
다른 물체의 성공률이나 실제 파지 성공은 검증하지 않았다.

원본은 `jetson-orin:/home/cleany/cleany-data/reports/arm-selection-fix-20261023-041756/`과
`arm-selection-verification-20261023-042645/`에 보관한다.
`comparison-summary.json`, `comparison.png`와 `report.html`은 비교 결과,
`implementation.patch`와 `change-manifest.json`은 이번 변경 및 전후 hash,
`baseline/`, `production/`, `production-repeat/`는 입력 CDR, IK 진단과 계획 결과,
XML과 빌드 로그는 검사 결과, `deployment.json`은 제어 프로세스 유지 기록이다.
초기 실패와 적용하지 않은 실험 로그도 같은 경로에 보존했다.

## 2026-10-23 DDS 수신 버퍼 대기 상태 비교

Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, CycloneDDS 0.10.5에서 commit
`3950218d70cbebd4ce3278e7602bc276482c0471`과 미커밋 작업 내용으로 측정했다.
운영 런타임은 `runtime/collection-operator/20261023-005421`이며, 01:16–01:23 KST에
별도 호스트 Best Effort 계측기의 수신 소켓 요청값을 208 KiB, 512 KiB, 1 MiB,
2 MiB, 4 MiB, 8 MiB 순서와 역순으로 비교했다. 각 회차 35초 중 처음 5초를
제외하고, 모터 집계 토픽 2개, 헤드 영상과 CameraInfo 4개, 양손목 영상과
CameraInfo 4개를 기록했다. 1초 이동 구간 Hz를 0.1초 간격으로 보존했다.

| 요청 버퍼 | 헤드 컬러 이동 구간 Hz 범위, 두 회차 | 계측기 UDP 유실, 초기 연결 포함 두 회차 합계 |
|---|---:|---:|
| 208 KiB | 0–15 | 23,789 |
| 512 KiB | 12–16 | 936 |
| 1 MiB | 13–16 | 30 |
| 2 MiB | 14–16 | 0 |
| 4 MiB | 14–16 | 0 |
| 8 MiB | 14–16 | 0 |

계측기 자신의 소켓 유실과 호스트 전체 `RcvbufErrors` 증가량이 모든 회차에서
일치했다. Linux `SO_RCVBUF`는 요청값의 두 배로 확인했다. 모든 위치 메시지에
14개 관절이 있었고 양손목은 OFF였다. 12회 모두 기록기 유실이 없었으며,
유실 0인 가장 작은 관측 요청값은 2 MiB였다. 운영 인식의 Reliable 수신기와
OS `rmem_max`는 8 MiB를 유지했으므로 운영 최소값이나 파지 중 안정성을 확정한
결과는 아니다. 작은 버퍼의 패킷 유실은 의도한 실험 결과이며 프레임 유실률과 다르다.

원본 위치는 데이터 루트의 `reports/dds-buffer-idle-sweep-20261023/`였으며,
2026-10-23 사용자 요청으로 다른 과거 Hz 기록과 함께 원본, 그래프와 보고서를
삭제했다. 위 수치는 당시 확인한 요약이며 현재 로컬 원본으로 재검증할 수 없다.
재실행 절차는 [테스트 가이드](TESTING.md#dds-수신-버퍼-단계-비교)를 따른다.
도구 검사 3개와 기존 IO 기록기 검사 9개가 통과했다. 실제 파지 시험은 미실시다.

## Jetson 실물 마운트와 카메라 보정

2026-10-21에 `jetson-orin:/home/cleany/cleany`의
`52e44abe89b7103bd832f3cbf806a121c37cb82c`와 미커밋 작업 내용을 읽어 선별했다.
프로파일은 [jetson_20261009](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/)
아래에 보존한다. [provenance.json](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/provenance.json)은
보정 원본, 사용한 내부 보정, 과거 검증 보고서의 경로와 SHA-256을 기록한다.
검증 지표와 실패 또는 미검증 조건은 각 extrinsics YAML과 [캘리브레이션 가이드](CALIBRATION.md)에 있다.
보고서 원본과 이미지 데이터셋은 Jetson의 기록된 위치에 남아 있다.

[mount.png](assets/physical-calibration/mount.png)는 사용자가 제공한 형상 렌더다.
`/Users/hyeon/Downloads/Jetson/mount.png`에서 원본 그대로 복사해 이전 PR 이미지를
교체했다. SHA-256은 `c36c8aff0047bbe3dd34ec6904da879269c4221d2dd68f64d0363cce8d5d9d8b`다.
실물 제어와 calibration 재수행의 증거로 사용하지 않는다.

### 2026-10-21 적용 검증

검증 코드는 `19f3882918f660fe580fe7f7fac0d8869d0a6331`이다.
Jetson Ubuntu 22.04, ROS 2 Humble, Python 3.10.12, MuJoCo 3.11.0의 별도
`/tmp/cleany-calibration-pr-20261021` workspace에서 실행했다.
`cleany_description`의 Git 추적 파일 100개가 해당 commit과 동일한지 hash로 확인했다.
빌드 대상은 description과 인접 MoveIt config 두 패키지이며 전체 workspace 검증은 아니다.

| 검사 | 결과 |
|---|---|
| 루트 `make build`, 별도 `CLEANY_BUILD_ROOT` | 두 패키지 빌드 통과 |
| `colcon test`: description 모델 정합성 | 46개 통과 |
| `colcon test`: 카메라 보정 프로파일 | 10개 통과 |
| MoveIt 설정과 collision 계약 | 18개 통과 |
| 기본 description launch | 명목 헤드 TF, 손목 보정 TF 미발행 확인 |
| `jetson_20261009` description launch | 보정된 헤드와 양손목 TF, RGB alias 5개 확인 |
| MoveIt mock 실행 | Pilz LIN plan-only 결과가 `ABORTED`여서 1개 실패 |

마지막 실패는 수정 전 `main`의 `d5a640a556d473b2a0af59932974b3b3b0aa4732`를
별도 workspace로 빌드해 동일 환경에서 재현했다.
`test_per_arm_position_ik_plan_and_execute`의 `_assert_pilz_lin_is_cartesian`에서
`status == 6`으로 실패하며 기대값은 `STATUS_SUCCEEDED == 4`이다.
이번 변경에서 실행기를 수정하거나 해당 검사를 생략하지 않았다.
Description launch 검사는 localhost 전용 ROS domain 212와 213에서 수행했고
실물 장치 명령은 전송하지 않았다.

원본 로그와 JUnit은 데이터 루트의
`reports/physical-calibration-20261021/`에 보관한다.
Jetson 원본 출력은 `/tmp/cleany-calibration-pr-20261021-output/`,
비교 기준 출력은 `/tmp/cleany-calibration-main-baseline-output/`이다.

### 2026-10-21 시뮬레이터 정합성 재검증

검증 코드는 `e5b26da2e077007003999d1abedb3b67447f6cba`다.
같은 Jetson 환경에서 `make build`로 두 패키지를 다시 빌드하고,
description의 모델 정합성 46개와 보정 프로파일 10개를 통과했다.
검사한 description 추적 파일 100개는 로컬 코드와 SHA-256이 일치한다.
두 URDF 모두 Jetson 모델의 마스트 간격 `0.05`를 유지하며,
`0.08`을 요청하면 오류로 거부하는 회귀 검사를 포함한다.

Gazebo의 `test_cad_frame.py` 6개도 통과했다. SDFormat 12.9.0과 스키마는
공식 ROS APT 패키지를 별도 `/tmp` 경로에 풀어 사용했다.
실제 URDF→SDF 변환 결과에서 RGB와 depth 센서의 위치 및 방향이
변경된 헤드 장착 형상과 TF metadata에 일치하는지 확인했다.
이 검증은 모델 변환 검사이며 Gazebo 동적 실행이나 실물 구동 검증은 아니다.
원본 로그와 JUnit은 데이터 루트의 `reports/pr58-fix-20261021/`에 보관한다.

### 2026-10-21 v8 손목 마운트 반영 검증

검증 코드는 `b6d52578f8e18f9a082cbb43cb2e56f6d8ebd902`다. Jetson의 같은
HEAD와 최신 미커밋 작업에서 v8 마운트만 선별했다. 메시 16개, 형상 JSON,
독립 MJCF와 xacro는 원본 바이트와 일치한다. 실제 보정 YAML 두 개도 이전
수집본과 동일하며, 이번에 hand-eye 보정을 다시 수행하지 않았다.

Ubuntu 22.04 / ROS 2 Humble / Python 3.10.12 / MuJoCo 3.11.0에서 별도
`/tmp/cleany-pr58-wrist-20261021` 사본과 `/tmp/cleany-pr58-wrist-output` 빌드
경로를 사용했다. Description 파일 121개가 로컬 코드와 SHA-256으로 일치한다.
루트 `make build`로 description과 MoveIt config 빌드를 통과했고,
`colcon test`에서 모델 정합성 46개, 보정 프로파일 10개, 마운트 자산 검사 1개,
MoveIt 설정 및 collision 계약 18개가 통과했다. 검사에는 좌우 30개 충돌
primitive의 배치와 크기, m/mm 단위, 실측 기준 치수와 원본 hash를 포함한다.

MoveIt mock 실행 1개는 앞서 `main`에서도 재현한 동일 Pilz LIN 단계에서
`ABORTED`로 실패했다. 이번 검사의 전체 colcon 결과는 이 실패를 포함한다.
새 마운트의 실물 간섭 여유, 카메라 정확도와 로봇 동작은 검증하지 않았다.
원본 로그, JUnit과 파일 manifest는 데이터 루트의
`reports/pr58-wrist-20261021/`에 보관한다.

### PR 58 카메라 수정 후속 검증 (2026-10-22)

기준은 PR 58의 `f8ffd75f741b8da9087de87b01d4ad5ea968ea37`과
`fix/pr58-wrist-camera-pose`의 미커밋 변경이다. MuJoCo의 렌더링 카메라와
optical body/site, sorting 카메라 설정을 v8 마운트 등록 위치·방향에 맞췄다.
Gazebo 레거시 및 팔 포함 CAD 모델도 같은 등록을 사용한다. 손목 마운트의
시야 가림과 좌표계 회귀 검사를 패키지 테스트로 등록했다.

CI 최적화는 PR 61에서 관리하며, 이 변경은 workflow, Makefile과 CI 인프라를
수정하지 않는다. 기존 모델 검사의 xacro 캐시와 Base 검사 등록 변경도 포함하지
않는다. 재촬영은 `tools/validation/capture_cameras.py`를 명시 실행한다.

Ubuntu 22.04 arm64, ROS Humble에서 별도 `builds/pr58-camera-only/` 경로에
관련 6개 패키지를 빌드했다. 최초 선택 빌드는 설치되지 않은 Navigation의
package 환경 파일 때문에 실패했다. Navigation C++ 의존성은 기존과 같이
충족되지 않아 선택 빌드에서 제외하고 촬영에 필요한 Python 소스만 PYTHONPATH로
제공했다. 최초 pytest도 description overlay를 source하지 않아 실패했으며,
새 빌드 overlay를 적용한 최종 description 65개, Gazebo 손목 2개와 도구 9개가
통과했다. 전체 workspace 빌드와 주행 검증을 뜻하지 않는다.

Xvfb/GLX와 software rendering에서 MuJoCo 및 팔 포함 Gazebo CAD의 실제
head/양손목 카메라 촬영을 통과했다. Xvfb는 기존 `pr58-test-cleanup/xvfb-bin`
링크로 추출된 실행 파일을 사용했다. 이 수정은 최신 main `254ffc4`와 함께
임시 worktree에서 충돌 없이 적용됐으며, 원격 CI는 실행하지 않았다.
원본 실패·성공 로그, JUnit, 카메라 PNG와 JSON, 변경 manifest는 데이터 루트의
`reviews/pr58-camera-only/`에 있다. 앞선 전체 backend 검증 원본은
`reviews/pr58-all-backends/`, CI 통합 비교 원본은 `reviews/pr58-pr61-preserved/`에
별도로 보존한다.

| 데이터 루트 아래 경로 | 내용 |
|---|---|
| `fixtures/cad-fortress-command-hold/` | 과거 시나리오 입력 |
| `fixtures/cad-fortress-command-hold-portable/` | local mesh/texture를 고정한 별도 입력 |
| `runs/scenarios/<run-name>/` | 과거 시나리오 결과, journal, 로그 |
| `runs/historical/20261010/artifacts/` | 이전 실행 원본 |
| `reports/legacy-20261010/docs/` | 과거 검증, navigation 분석, ChatGPT 인수인계 사본 |
| `experiments/legacy-20261010/` | 과거 캠페인, SLAM 코드 사본 |
| `experiments/legacy-runtime-cleanup-b3ca816/manifest.json` | 레거시 코드의 commit/hash |
| `migrations/20261010-repository-cleanup/` | 이관 대응표, hash, 검증 로그 |
| `archives/legacy-20261010/scrum-426-scenarios-20261010/worktree.tar.zst` | 이전 scenario worktree |
| `presentation/legacy-20261010/` | 발표 참조, 원본 자료 |
| `builds/<worktree-name>/` | 해당 worktree의 colcon 출력 |
| `builds/ci-optimization-evidence/` | CI 실행 metadata와 원본 로그, 테스트 수집 비교, 종료 경쟁 재현 |
| `reports/documentation-reorganization-20261013/before/` | 이번 문서 개편 전 Markdown 사본 |
| `reports/documentation-reorganization-20261013/manifest.json` | 사본 source commit, dirty 상태, 각 파일 SHA-256 |

## 재현, 보존

문서 개편 전 사본의 source commit은 `fbe5854`입니다. Commit, dirty 상태, SHA-256은 보관 manifest, 과거 경로 대응은 `relocation-manifest.json`에서 확인합니다. 원본의 절대 경로와 hash는 보존하고, fixture의 현재 위치와 profile-manifest의 과거 provenance를 구분합니다.

재현할 때는 기록된 commit의 별도 checkout과 해당 환경, 입력을 사용합니다. 다른 브랜치의 자료는 해당 버전의 근거로 취급하며, 제품 결정은 KB 검토, Decision 절차를 따릅니다.

## Mission Mock 조작 통합

| 코드와 환경 | 입력과 결과 범위 | 원본 (`cleany-data/runs/` 아래) |
|---|---|---|
| `171ac80`, Ubuntu 22.04 ARM64, Humble/Fortress | 40초 공유 fixture, study-cafe seat-12, wait_for_next. Mission/Mock/Bridge ROS 208, Monitor 41, 웹 27 검사 통과. Gazebo 139.97초, 두 execution ID와 추가 빈 관측. 깨끗한 Nav2 재빌드는 삭제된 임시 dependency export 경로로 실패했으며 실행은 기존 설치 사용 | `code-review-20261020/verification.json`, `execution-evidence.json`, 로그와 DB |
| 위 코드의 문서 정리 | 링크, 명령과 소스 불변 검사. 최신 #54 병합 전 근거 | `docs-review-20261020/` |
| 이전 40초 fixture | 초기 시간 조정과 ExecuteOne 관측 자료 | `manipulation-timing-20261015-XwwCUn/`, `code-verification-20261015-NS9Nqz/` |

이 수치는 서로 겹치는 검사 묶음이며 합산하지 않는다. 현재 코드의 근거와 과거 실행을 구분한다.

### 최신 PR #54 병합 검증

`171ac80`과 PR #54 `65fdf06`의 병합 작업트리를 Ubuntu 22.04 ARM64, Humble/Fortress에서 검사했다. 원본은 `cleany-data/runs/merge-pr54-20261020/`에 보관한다. 최종 commit과 세부 결과는 해당 폴더의 `verification.json`을 기준으로 확인한다.

| 검사 | 입력과 결과 | 원본 |
|---|---|---|
| Mission과 Mock Action | 두 물체 fixture, ROS domain 193. 208 passed, Gazebo 1개는 별도 실행 | `ros-mock.log` |
| Monitor | ROS domain 194. Cartographer ROS 1개를 제외한 99 passed. 전체 실행의 해당 1개는 `cartographer_ros_msgs` 미설치로 import 실패 | `monitor-ros.log`, `monitor-ros-available.log` |
| 웹 fixture | 33 passed, 실제 Monitor가 필요한 3개는 별도 실행. TypeScript와 production build 통과 | `web-all.log`, `web-build.log` |
| Gazebo와 Mock 조작 | 새 통합 install, study-cafe seat-12, domain 195, wait_for_next. 138.51초에 SUCCESS. 서로 다른 execution ID 2개, 각 17개 세부 동작과 추가 빈 관측 확인. 두 DB 무결성 통과 | `gazebo.log`, `gazebo-evidence.json` |
| 실제 Monitor | FSM, 토픽 선택, 기록 재생, host resource와 Gazebo 지도 검사 통과. 실제 조작 화면과 ExecuteOne 상세도 확인 | `web-live.log`, `web-map.log` |
| 새 통합 build | 생성물 정리 후 16개 package와 Monitor 설치 build 통과 | `build-sim-clean.log`, `build-monitor-clean.log` |
| 계약과 문서 | 최신 #54 FSM, 청소 BT, journal과 보고 model 동일. 로컬 문서 링크 146개 확인 | `mission-core-invariants.json`, `docs-check.json` |

로컬 Nav2 cache의 CMake export가 예전 `/tmp` 추출 경로와 누락된 include 경로를 참조했다. 검증에서는 원래 prefix를 현재 cache에 연결하고 `CPLUS_INCLUDE_PATH`로 설치된 header root를 제공했다. 생성된 export 파일은 수정하지 않았다. 이 환경 보정은 새 VM의 의존성 설치를 대체하지 않으며 재부팅 후 임시 prefix를 다시 준비해야 할 수 있다. 상세 경로와 include 목록은 `nav2-environment.json`, `nav2-build-includes.json`에 남겼다.

### 병합 후 코드 검토

`a7317be` 이후 작업트리를 Ubuntu 22.04 ARM64, Humble/Fortress에서 검토했다. 원본은 `cleany-data/runs/quality-review-pr54-20261020-08o9nj4k/`에 보관하며 최종 소스 hash와 집계는 `verification.json`을 따른다.

| 검토 항목 | 반영 내용 |
|---|---|
| Action remap | Humble의 client와 server에서 이름을 명시적으로 해결하고 내부 CancelGoal 경로도 맞춤. 기본 주소와 사용자 주소에서 두 물체 실행 및 취소 검사 |
| 종료 중 DB 쓰기 | callback과 같은 잠금으로 DB를 닫고 종료 후 tick을 차단. 진행 기록을 ACTIVE로 보존하며 종료를 정지 증거로 만들지 않음 |
| 요청 검증 | 문자열이 아닌 ID와 bool 또는 소수 object ID를 core 수락 전에 거절 |
| 중복과 테스트 | 사용되지 않는 저장소 복구 함수와 미사용 import 제거. 복구 정책은 기존 ExecutionJournal에서 수행. 합성 검증 timeout 대기를 5초에서 2초로 축소 |
| 웹 검사 격리 | UI fixture의 기록 API를 명시하여 실제 로컬 Monitor에 의존하지 않도록 수정 |
| 계약과 문서 | 서버 wall 시각과 steady timeout 구분, 조회와 DB 저장 성공의 차이를 명시. FSM, 청소 BT, 보고 model과 ROS 타입 23종 유지. 웹과 서버의 정상 9단계, 17동작 일치 |

초기 `remap-before.log`와 `ids-before.log`에서 기존 실패를 확인했다. 첫 전체 ROS 실행은 종료 중 SQLite 쓰기와 닫기가 겹쳐 segfault로 중단됐고 `ros.log`에 보존했다. 종료 동기화 수정 후 해당 회귀 검사와 서버 검사 34개가 통과했다. 최종 전체 ROS 검사는 **216 passed, 1 skipped**이며 Gazebo 1개는 별도로 통과했다. 원본은 `ros-final.log`다. 문서 링크 146개와 Python 소스 31개의 syntax, Pyflakes를 확인했다.

Monitor는 99개, 웹 fixture는 33개가 통과했으며 변경한 UI fixture 6개도 다시 통과했다. Cartographer ROS 1개는 이전과 같이 `cartographer_ros_msgs`가 없어 제외했다. Gazebo는 142.52초에 두 execution ID의 수거와 각각 17동작, 추가 빈 관측을 확인했고 Mission DB와 서버 DB의 integrity_check가 모두 통과했다. 마지막 종료 잠금 수정은 별도 동시성 회귀 검사로 검증했다. 원본은 `monitor.log`, `web.log`, `ui-final.log`, `gazebo.log`, `gazebo-evidence.json`, `stage-contract.json`, `static-final.log`에 있다.

## CI 성능 검증 (2026-10-20)

기준은 Ubuntu 22.04 GitHub runner, ROS Humble의 [실행 39417986298](https://github.com/asm17-moving-agent/cleany/actions/runs/39417986298), commit `11d1ba8`입니다. 전체 job은 804초, workspace 의존성 설치 288초, 빌드와 테스트 279초, 별도 Navigation 빌드와 runtime 123초였습니다. 입력은 해당 commit의 전체 workspace 검사와 기존 motor, Navigation, Gazebo runtime 검사입니다.

| 실행 | Commit | Job 전체 | 결과 |
|---|---|---|---|
| 기존 기준 | `11d1ba8` | 13분 24초 | 통과 |
| [이미지 생성, 게시와 compiler cache 최초 준비](https://github.com/asm17-moving-agent/cleany/actions/runs/39426732345) | `d44b065` | 15분 5초 | 통과 |
| [이미지와 compiler cache 재사용](https://github.com/asm17-moving-agent/cleany/actions/runs/39429236181) | `45df20f` | 4분 37초 | 통과 |

재사용 실행은 기준보다 65.5% 짧았습니다. 이미지 pull 84초, cache 복원 2초, 전체 workspace 빌드와 테스트 90초, motor core 1초, Navigation runtime 30초, Gazebo runtime 46초가 포함됩니다. C++ 입력이 같은 상태에서 compiler cache는 264건 모두 적중했고 전체 빌드는 42.7초였습니다. Workspace 검사 집계는 1,882개, 실패 0개, 기존과 같은 skip 24개이며 CI 도구 9개, Navigation runtime 3개와 Gazebo runtime 1개도 통과했습니다. 베이스 검사를 직접 보고하도록 바꾸어 기존 중첩 래퍼가 집계하지 않던 검사도 XML에 포함됩니다.

5분 이내는 위 cache 적중 실행의 실측값입니다. 의존성 변경이나 주차 갱신으로 이미지를 다시 만들거나, C++ 변경으로 cache miss가 생기면 더 오래 걸릴 수 있습니다. 캐시가 없는 최초 실행도 전체 검사를 통과했습니다.

`1ab6d8a`의 [최초 실행](https://github.com/asm17-moving-agent/cleany/actions/runs/39424741384)은 기존 command guard SIGINT 종료 검사에서 메시지 변환 중 RuntimeError로 실패했습니다. `d44b065`에서 콜백 이후에 종료하도록 수정했습니다. 동일한 Humble 의존성 이미지의 Linux arm64 로컬 재현은 수정 전 10회 중 3회 실패, 수정 후 기존 검사 30회 연속 통과였습니다. 베이스 드라이버의 기능 검사 수집 ID 72개는 변경 전후 동일하며, 기존 모델 검사 함수 24개와 runtime 검사 파일을 유지했습니다.

`d5852af`의 [환경 확인 실패](https://github.com/asm17-moving-agent/cleany/actions/runs/39428868069)는 런타임 `ROS_HOME`이 이미지의 rosdep metadata 위치와 달라 발생했습니다. `45df20f`는 의존성 확인 명령에 이미지 생성 시의 `ROS_HOME`을 지정하며, 재설치 없이 실제 manifest의 의존성 충족 여부를 검사합니다.

원본은 데이터 루트의 `builds/ci-optimization-evidence/`에 보존합니다. `timing-comparison.json`, `baseline-39417986298.json`, `cold-*.json`, `cold-*.log`, `warm-*.json`, `warm-*.log`, `coverage-audit.json`, `base-*-collection.txt`, `shutdown-before.log`, `shutdown-after.log`에서 환경, 실패와 검사 범위를 확인합니다. `cold-artifacts/`, `warm-artifacts/`와 CI artifact는 test XML, apt/pip 버전과 compiler cache 통계를 보존합니다. Simulator와 합성 backend 검사이며 실물 로봇 검증을 뜻하지 않습니다.


## Jetson 변경 선별 검증 (2026-10-22)

기준 main은 `d0afd48`, 검증한 구현은 `integrate/jetson-main-curated`의 `d81212e`다. 원본 `feat/jetson-manipulation`의 `9c85b7d`와 미커밋 변경은 별도 snapshot으로 보존했다. 원래 작업 폴더의 변경 파일 364개 SHA-256과 전체 Git 상태가 백업 당시와 일치했다.

Ubuntu 22.04 arm64, Python 3.10, ROS Humble, MoveIt 2.5.9와 geometric_shapes 2.3.4에서 검사했다. 기존 사용자 prefix의 scene runtime, OctoMap lock overlay와 BehaviorTree.CPP 4.10.0을 사용했으며 빌드 출력은 별도 `cleany-data/builds/cleany-main-curated/`에 뒀다. 입력은 합성 RGB-D와 transport, 가상 모터 PTY, 저장소의 고정 보정과 모델이다.

| 검사 | 결과 | 원본 로그 |
|---|---|---|
| 관련 native 빌드 | 14개 패키지 통과 | `build-final-3.log` |
| Interfaces, perception, grasp, physical state, skill와 BT 회귀 | 3150 passed, 28 skipped | `package-tests-verified.log` |
| Host 도구 | 235 passed, 3 skipped | `host-tools-tests-final.log` |
| 설정 정리 후 TCP 도구 | 36 passed; 선택하지 않은 SDK 검사 3 skip | `tcp-tools-tests-final.log` |
| Controller의 READ 전용 활성화 차단과 쓰기 부재 | PTY 2개 통과 | `controller-emulator-tests-2.log` |
| 모델 parity, 보정, v8 mount와 collision mesh | 74 passed | `model-tests-final.log` |
| Scene mapping, Feetech driver와 MoveIt capability C++ | 3개 패키지 통과 | `native-tests.log` |
| Monitor Python | 154 passed, 17 skipped | `monitor-tests-final.log` |
| Monitor 웹 fixture | 44 passed, 3 skipped | `web-e2e-tests.log` |
| GPU export와 runtime 경계의 장치 없는 검사 | 34 passed | `vision-tools-tests-final.log` |

초기 통합에서 기존 main의 API와 launch 설정, 점군 buffer 검증, 로컬 전용 profile 참조와 보정 fixture의 경계값이 실패했다. 별도 physical 진입점, 명시적 외부 설정, 완전한 buffer 검사와 버전 관리되는 보정 입력으로 수정했다. 삭제한 legacy 파일의 이전 build symlink는 새 작업 폴더의 출력에서만 정리한 뒤 다시 빌드했다. 기존 시뮬레이션, Navigation, Mission과 CI 원본은 유지했다.

원본 로그, 설정별 판정, 전체 파일 선별 내역, 이전 실패와 보존 검사는 `cleany-data/reviews/jetson-main-curation-20261021T173354Z/`의 `verification.json`, `file-dispositions.tsv`, `source-preservation.json`과 위 로그에 있다. NumPy 1.26.1과 시스템 SciPy의 버전 경고가 있었으며 관련 검사는 통과했다. 선택형 GPU, LeRobot, licensed SDK와 별도 ROS 시나리오는 실제 실행 조건에 따라 skip했다. 실제 USB 장치와 모터를 구동하지 않았고 GitHub CI, 실물 안전과 작업 성공률을 검증한 결과는 아니다. 변경된 profile lock의 기존 engine manifest는 별도 export와 검증이 필요하다.


## AnyGrasp 강제 GC 주기 비교 (2026-10-22)

기준은 Jetson의 `3950218d70cbebd4ce3278e7602bc276482c0471`과
`cuda_gc_interval_calls` 기본값을 16으로 바꾼 미커밋 변경이다.
Jetson Orin NX 16GB, 25W 모드와 기존 `cleany-vision:jp6.2` 이미지에서
입력 8,000점, CUDA 미사용 캐시 반환 기준 512MiB를 유지했다.
자동 Python GC는 유지하고 명시적 전체 GC만 최소 16회 추론 간격으로 실행한다.
값 1은 기존 매회 수집 동작이다. 설정은
[predictor 계약](../ros2_ws/src/cleany_grasping/docs/predictors.md)을 따른다.

기존 OCID fixture manifest의 유효 점군 91개를 두 독립 worker에서 순차 실행하고
장면마다 실행 순서를 바꿨다. 원본 100개 중 이전 phase2 처리 실패로 제외된 9개는
이번 시험 입력에 포함되지 않는다. worker별 최초 3회는 준비 구간으로 제외했다.
재시도와 후보 미생성 요청을 포함한 평균은 2.292초에서 2.055초로 10.3%,
p95는 4.916초에서 4.554초로 7.4% 줄었다. 두 방식 모두 13회 재시도했고
78개 장면에서 후보를 생성했다. 91개 장면의 후보 수와 소수점 6자리로 반올림한
원시 출력 hash가 일치했다. 추가 256회, 약 8.6분 연속 실행에서 처음 및 마지막
16회 평균 RSS 차이는 +0.36MiB였고 추론 후 CUDA 예약 메모리는 120MiB였다.
이 실행 길이로 장시간 메모리 안정성을 확정하지 않는다.

루트 Makefile로 interfaces와 grasping 두 패키지를 별도 경로에 빌드했고,
grasping 테스트 109개, ROS 설정 전달과 실제 SDK worker 재사용 및 종료 검사가
통과했다. 카메라, ROS 서비스 전송, VLM, IK와 실물 파지 성공률 검증은 아니다.

원본은 `jetson-orin:/home/cleany/cleany-data/experiments/anygrasp-gc-20261022/`에
보관한다. `metadata.json`과 `patch-manifest.json`은 환경과 소스 hash,
`fixtures-manifest.json`은 입력 및 제외 항목, `requests.jsonl`과
`tegrastats.log`는 측정 원본, `validation.json`과 `pytest.xml`은 빌드 및 검사
결과를 기록한다. `report.html`, `comparison.png`와 `latency.png`는
`summary.json`에서 생성한 비교 보고서다.

## 들어올리기 높이 보정 1회 전환 검증 (2026-10-22)

기준 commit은 `3950218d70cbebd4ce3278e7602bc276482c0471`이며 미커밋 높이 보정 변경을 포함한다. Jetson의 native ROS 2 Humble, Python 3.10과 기존 motion container에서 검사했다. 입력은 실측 높이 부족, 일정한 하중 처짐, 보정 후 높이 정체, 기존 3회 보정 설정, 양팔의 BT 및 기존 pipeline 합성 motion port다. 11mm 부족분의 15mm 단일 보정, 잔여 높이 오차에 의한 중단 없이 복귀와 놓기 진행, 기존 피드백 및 경로 실패의 전파를 확인했다.

관련 pytest 783개가 통과했고 최종 검사 실패는 없다. 확장한 합성 fixture는 최초 실행에서 도착 관절 허용값 입력이 누락되어 8개가 실패했으며 해당 입력을 보완했다. `make build-manipulation-bt`의 13개 패키지 빌드와 `make motion-container-prepare`의 5개 패키지 빌드 및 import 검사가 완료됐다. Container 빌드에는 Git metadata를 찾지 못한 setuptools_scm 메시지가 있었지만 빌드는 성공했다. 실제 모터 명령은 전송하지 않았으며 실물 파지 성공을 검증한 결과는 아니다.

원본은 `/home/cleany/cleany-data/reports/lift-single-correction-20261022/`의 `pytest.log`, `pytest.xml`, `build.log`, `motion-prepare.log`에 보관한다. 동작 계약은 [실물 조작 경로](../ros2_ws/src/cleany_skill_executor/docs/physical-manipulation.md)를 따른다.

## AnyGrasp 입력, 사전 준비와 작업 유지 비교 (2026-10-22)

기준 commit은 `3950218d70cbebd4ce3278e7602bc276482c0471`이며 앞서 적용한
GC16과 기존 작업트리를 포함한다. Jetson Orin NX 16GB, 25W, native Humble과
실제 licensed AnyGrasp 및 cuRobo 0.7.8 GPU container에서 검사했다.
변경 전후 파일 hash와 GC16 이후 증분 변경은 `pipeline/change-manifest.json`과
`pipeline/implementation.patch`를 따른다. 관련 없는 기존 수정은 보존했다.

기존 OCID manifest 유효 장면 91개에 대해 8,000점 기준, 접근 방향 60도,
target voxel ROI, 고정 4,000점과 가변 4,000/8,000점을 각각 독립 worker에서
실행했다. 장면마다 실행 순서를 바꿨으며 worker별 준비 3회는 제외했다.
원본 100개 중 이전 phase2 실패 9개는 기존 manifest와 동일하게 제외했다.
재시도와 후보 미생성을 포함한 평균은 기준 2.054초, 방향 1.962초,
ROI 1.994초, 4,000점 1.357초, 가변 1.740초였다. 각각 4.5%, 3.0%,
33.9%, 15.3% 감소다. 후보를 생성한 장면은 78, 82, 82, 80, 78개지만
방향은 기존 장면 0018과 0067에서, ROI는 0051에서 후보를 잃었다.
가변 정책은 43개 장면에서 4,000점을 선택했다. 접근 방향은 저장 장면의
지지면 추정값이며 실물 로봇 도달성 검증이 아니다. 입력 조건 기본값은
제한 없는 방향, workspace ROI와 고정 8,000점으로 유지했다.

사전 준비는 실제 notifier와 두 GPU 모델을 사용하되 VLM을 3, 15, 30초
시간 대기로 대체하고 ROS 전달을 파일과 스레드로 재현했다. 공통 geometry
cache를 준비하고 조건별 새 process에서 각각 2회 측정했다. VLM 종료 후
후보 추론까지 평균은 10.766→7.195초, 10.094→4.350초, 10.241→2.504초다.
VLM 시작부터 후보와 cuRobo 준비 및 notifier 정리까지 평균은
26.115→26.434초, 25.114→26.163초, 40.259→32.561초였다.
30초 대기에서는 전체 준비가 19.1% 줄었지만 짧은 대기에서는 소폭 늘었다.
AnyGrasp 상주 대조군 1회에서도 후보까지 1.596→2.389초로 늘어,
모든 조건에서 이득이 있다고 판단하지 않는다. cuRobo는 현 로봇 geometry와
합성 관절 위치를 사용했고 실제 물체의 IK 및 경로 계획은 측정하지 않았다.

모델 유지 비교는 idle 120초를 유지한 채 실제 125초를 기다렸다. 기존은
worker를 해제해 다음 요청 9.848초, 작업 lease 600초는 동일 PID를 유지해
1.603초였다. 대신 worker RSS 약 1.18GiB가 상주한다. 이는 시스템 전체
추가 RAM과 같지 않다. 가용 메모리보다 높은 임계값으로 압박 해제 분기를
검사했으며 실제 RAM 고갈 시험은 아니다. 메모리 임계값 기본 0은 비활성이다.
입력 비교에서 worker별 RSS 절대값 차이가 있었지만 초기 상태 차이도 포함하며
91회 동안 지속 증가를 확인하지 않았다. 4,000점의 요청 중 CUDA 예약 최대값
중앙값은 1,904→1,088MiB였지만 전체 해상도 재시도로 최댓값 약 3.45GiB가 남았다.

배포 설정에 AnyGrasp 후 cuRobo 준비, 공유 GPU 작업 순서와 실제 요청 우선권,
VLM 및 작업 상태의 유한 시간 lease를 적용했다. native 관련 3개 패키지 빌드,
pytest 241개, motion 이미지 및 5개 패키지 준비, vision overlay 빌드가 통과했다.
실제 ROS service와 licensed worker로 준비, IK 알림 순서, 종료 정리, 작업 상태
유지와 반복 종료 상태 무시도 확인했다. 최초 ROS 검사는 interface overlay를
source하지 않아 실패했고 overlay 빌드 및 source 후 통과했다.
카메라, 실제 VLM API, 실제 물체 IK, 로봇 구동과 파지 성공률 검증은 아니다.

통합 보고서와 그래프는
`jetson-orin:/home/cleany/cleany-data/reports/anygrasp-optimizations-20261022/`의
`report.html`에 있다. `options/`는 입력 비교 455개 요청 원본과 환경,
`pipeline/raw/`는 사전 준비 14개 사례 및 모델 hash,
`pipeline/retention/`은 실제 125초 유지 및 압박 분기 원본,
`pipeline/validation.json`, XML과 로그는 빌드 및 검사 결과다.
`summary.json`, `options/measurements.csv`와 `plot_suite.py`에서 그래프를 재생성한다.

## manipulation-run AnyGrasp 후보 전환 (2026-10-22)

기준 commit은 `3950218d70cbebd4ce3278e7602bc276482c0471`이며 기존 미커밋
AnyGrasp 최적화와 작업트리를 포함한다. Jetson Orin NX, native Humble과 기존
licensed vision 및 motion container에서 검사했다. `make manipulation-run`이
호출하는 `tools/collection-runtime`의 기본 후보 방식을 AnyGrasp로 바꿨다.
해당 모드의 hardware support launch는 기하학적 후보 노드를 생성하지 않는다.
명시한 `CLEANY_COLLECTION_GRASP_PREDICTOR` 값은 기본값보다 우선한다.
배포 YAML은 `target_geometry`로 4,000/8,000점을 선택하며 기존 전체 해상도
재시도, worker 재사용과 작업 유지 설정을 사용한다. 노드 자체 기본 정책과
접근 방향 및 ROI 제한은 변경하지 않았다.

운영 도구 pytest 103개와 launch 검사 8개가 통과했다. 도구 수집에서 선택형
3개가 skip됐으며 나머지 139개는 이번 선택 범위에서 제외했다. 기본 선택과
모드별 서비스 분기만 먼저 확인한 12개 검사는 위 도구 검사에 포함된다.
루트 Makefile로 grasping native 빌드와 motion container 준비를 마쳤다.
실제 `run-anygrasp` entrypoint의 predictor, 가변 정책과 worker 설정을 ROS로
조회하고 native client에서 `/grasp/plan`을 호출했다. 저장 fixture 0002,
0001, 0002에서 후보 29개, 21개, 29개를 반환했고 동일 worker PID를 유지했다.
저장 점군은 identity base_link frame으로 전달한 서비스 검사이며 카메라,
실제 물체 IK, 로봇 구동 및 파지 성공 검증은 아니다. 시험 container는 종료했다.

초기 launch 검사는 ROS Python 경로를 덮어써 collection 단계에서 실패했고
기존 overlay 경로를 유지해 통과했다. 서비스 검사는 고정 identity의 ROS domain을
바꾼 시험 설정이 preflight에서 거절됐으며 해당 identity는 수정하지 않았다.
같은 container 안의 별도 client도 정적 DDS 경로에서 서비스를 발견하지 못했다.
운영과 같은 native→container 및 기준 domain 0 경로로 검사해 위 요청이 통과했다.

원본은 `jetson-orin:/home/cleany/cleany-data/reports/manipulation-anygrasp-20261022/`에
보관한다. `implementation.patch`, `change-manifest.json`과 `verification.json`은
기존 수정 보존 및 변경 전후 hash, XML과 `validation.json`은 검사와 빌드 결과,
`ros-smoke.json`, `ros-smoke.log`, `anygrasp-server.log`는 실제 설정과 후보 응답을
기록한다. 초기 실패 로그도 같은 위치의 `*-before*` 파일에 보존했다.
