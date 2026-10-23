# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

## 2026-10-24 Dev Monitor 조작과 관측 통합 배치

로컬 `7908ef2`, Jetson `4f92bc0` 기반에서 조작 패널을 포함한 매니퓰레이션
배치를 검증했다. macOS의 Node.js 26.5.0, pnpm 11.28.8, Chrome에서
합성 WebSocket과 제어 API를 사용하는 브라우저 검사 15개가 모두 통과했다.
인증, 승인, 중단, 복구 확인, 기능 비활성 안내와 1440px, 1280px, 390px
배치를 확인했다. 라이트와 다크 화면을 검토했고 실물 조작 명령은 보내지 않았다.

`make build-dev-monitor-web`의 타입 검사와 웹 빌드, Ubuntu 22.04와
ROS 2 Humble을 사용하는 Jetson의
`make build COLCON_BUILD_ARGS='--packages-select cleany_dev_monitor'`가 성공했다.
테스트 실패는 없으며 Vite의 sourcemap, `use client` 무시와 500kB 초과
번들 경고는 남았다. 실제 서비스에서 조작 패널과 실행 요약의 좌우 배치,
인증 영역 접기와 펼치기, 가로 넘침 없음을 확인했다. 제공 중인 HTML, CSS와
JS는 검증한 빌드 산출물과 SHA256이 일치한다.

원본은 로컬 `output/dev-monitor-layout-20261024/`와 Jetson의
`../cleany-data/reports/dev-monitor-layout-20261024/`에 있다.
`verification.json`, `source-sha256.json`, `served-assets.json`,
`implementation.patch`, 빌드 및 검사 로그와 화면 캡처를 보존했다.
Jetson 적용 전 파일과 웹 산출물은 `before-layout.tar.gz`에 있다.

## 2026-10-24 Dev Monitor 매니퓰레이션 화면 가독성

로컬 `7908ef2`, Jetson `4f92bc0` 기반의 UI 변경을 macOS의 Node.js 26.5.0,
pnpm 11.28.8, Chrome과 Jetson의 Ubuntu 22.04, ROS 2 Humble에서 검증했다.
합성 WebSocket과 제어 API를 사용하는 매니퓰레이션 및 조작 제어 브라우저 검사
15개가 모두 통과했다. 진행, 실패, 오래된 관측, 재생과 승인 확인을 포함하며
라이트, 다크와 390px 화면을 확인했다. 실제 로봇 동작 시험은 수행하지 않았다.

`make build-dev-monitor-web`의 타입 검사와 웹 빌드, Jetson의
`make build COLCON_BUILD_ARGS='--packages-select cleany_dev_monitor'`가 성공했다.
테스트 실패는 없었고 Vite의 `use client` 무시 및 500kB 초과 번들 경고가 남았다.
실제 서비스에서 종료된 요청 이력의 접기와 펼치기, 실패 상태 표시와 가로 넘침
없음을 확인했다. 서빙되는 HTML, CSS와 JS는 빌드 산출물과 SHA256이 일치한다.

원본은 로컬 `output/dev-monitor-ui-20261024/`와 Jetson의
`../cleany-data/reports/dev-monitor-ui-20261024/`에 있다. `verification.json`,
`source-sha256.json`, `served-assets.json`, `implementation.patch`, 빌드 및 검사
로그와 화면 캡처를 보존했다. Jetson 적용 전 파일은 `before-ui.tar.gz`에 있다.

## 2026-10-23 Gemini 종류명과 YOLOE 원본 클래스 분리

로컬 `7908ef2`, Jetson `4f92bc0` 기반의 변경을 Jetson Orin NX,
Ubuntu 22.04, ROS 2 Humble와 Python 3.10.12에서 검증했다.
Gemini 합성 응답, 합성 RGB/mask와 ROS 메시지 직렬화로 종류명 교정,
동의어 정규화, 추적 ID 유지, 원본 클래스의 head/손목 전달 및 승인 거절을
확인했다. 실제 영상에 대한 Gemini 정확도나 실물 파지 시험은 아니다.

| 검증 | 결과 |
|---|---|
| 인식 패키지 및 승인 경계 통합 검사 | 332 통과, 1 제외 |
| Dev Monitor | 182 통과, 18 제외 |
| 6개 패키지 연동 회귀 | 수정 후 전체 재실행 3,282 통과, 28 제외, 실패 0 |
| 별도 ROS domain의 승인 통신 검사 | 1 통과; 합성 인식 결과 수신, 주변 객체 소실 허용, 대상 이동 거절 및 단일 실행 Goal 확인 |
| native ROS build | 5개 패키지 성공 |
| perception / AnyGrasp / motion 컨테이너 workspace | 각각 2 / 2 / 5개 패키지 빌드 성공 |

최초 6개 패키지 회귀는 3,277 통과, 28 제외, 5 실패였으며, 수정 후 실패 항목
5개와 전체 회귀를 각각 재실행해 통과했다. 초기 검사에서 기존 라벨 결합을 기대한 assertion, ROS float 필드의 정수 입력,
테스트용 관측 객체의 새 필드 누락을 수정했다. 프로세스 재시작 시험의 DDS
응답 시간 초과는 코드를 변경하지 않은 개별 및 전체 재실행에서 통과했다. motion 빌드의
`setuptools_scm` Git 파일 열거 경고는 남았지만 빌드와 설치 import 검사는 성공했다.

원본은 Jetson의 `../cleany-data/reports/semantic-labels-20261023/`에 있다.
`implementation.patch`, `verification.json`, `source-sha256.json`,
`package-tests.log`, `retry-package-failures.log`, `perception-integration-tests.log`,
`monitor-tests.log`와 각 build log를 보존했다. native와 컨테이너를 함께 갱신했다.
전체 재검증 근거는 `revalidation.json`, `package-tests-final.log/.xml`,
`ros-approval-final.log/.xml`, `implementation-final.patch`, `source-sha256-final.json`이다.
재검증 전에 적용 파일 37개가 최초 검증 시점과 동일함을 확인했다.
제외된 28개는 실물 카메라와 명시 실행이 필요한 GPU 및 런타임 검사다.
검증 당시 카메라 장치가 없고 운영 런타임은 중지 상태여서 실제 인식과 로봇
동작은 실행하지 않았다. 실행 절차는 [관측 인터페이스 갱신](RUNNING.md)을 따른다.

## 2026-10-23 충돌 형상 시각화

macOS의 `7908ef2` checkout과 Jetson의 `4f92bc0`에서 읽은 현재 URDF,
cuRobo geometry cache로 PNG 4장과 회전 가능한 3D HTML을 생성했다.
현재 URDF의 mesh 38개 인스턴스에서 원본 STL 정점은 260,261개,
최적화 STL 정점은 37,395개다. 같은 모델의 구는 병합 전 272개에서
병합 후 120개이며, 양팔만 집계하면 224개에서 96개다.

원본과 최적화 STL 16개 hash, fitting cache checksum, 병합 구의 기존 구 포함,
전체 모델 cache와 구 좌표 일치를 확인했다. PNG는 육안으로 확인했다.
비교용 기준 자세를 사용했으며 실시간 자세, 새 충돌 시험이나 속도 측정은 아니다.
Jetson 설치 overlay의 `camera_calibration.xacro` 누락으로 최초 확장에 실패해
같은 프로필로 checkout 소스를 직접 확장했다. 로봇 구동 명령은 보내지 않았다.

원본과 결과는 `output/collision-visualization-20261023/`의 `source/`,
`provenance.json`, `render.log`에 있다. 과거 비교 기록은 Jetson의
`runtime/mesh-optimization-20261019-stage2/review-results.json`과
`runtime/curobo-plan-comparison/sphere-optimization/optimization-summary.json`을
별도로 보존했다. 재생성은 [충돌 형상 시각화](TESTING.md#충돌-형상-시각화)를 따른다.

## 2026-10-23 기하학적 후보 생성으로 운영 전환

- **검증 환경**: Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble (Commit: `3950218` 기반)
- **변경 목적**: 공통 수거 런타임의 기본 파지 후보 방식을 `geometric`(기하학적 계산)으로 변경하여 `make manipulation-run` 및 Dev Monitor에 적용. (cuRobo IK/MoveIt 경로, 20mm 추가 접근 보정, 8/6/4cm 준비 위치는 유지)
- **주요 검증 결과**:
  - `make test-jetson-tools JETSON_PYTEST_ARGS="-k test_collection_runtime"` 실행 결과 103개 통과, 3개 제외.
  - AnyGrasp 컨테이너를 안전 종료하고 네이티브 기하학적 후보 서버가 유일한 `/grasp/plan` 제공자임을 확인.
  - Docker 네트워크 재생성 중 DDS 통신 일시 오류가 발생했으나, 하드웨어 복구 명령을 통해 추가적인 모터 쓰기 없이 정상 복구 완료.
  - 최종 상태: 양팔 및 그리퍼 컨트롤러 `active`, 안전 가드 `ready`, 실행기 오류 없음 확인.
  - 실물 카메라 관측: 파란색 물체(Object 2, 보조배터리) 대상 점군 173개(타깃) 및 3,760개(컨텍스트)로 53개 후보를 약 0.912초에 성공적으로 생성. (실제 팔 이동 명령은 보내지 않음)
- **산출물 및 로그 위치**:
  - 보고서 및 로그: `../cleany-data/reports/geometric-switch-20261023/` (`deployment.json`, `verification.json`, `implementation.patch`, `runtime-tests.xml`, `deployment.log`)
  - 카메라 및 점군: `../cleany-data/reports/geometric-camera-20261023-065858/`

## 2026-10-23 승인 대상만 비교하는 재검증

- **검증 환경**: Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble (Commit: `3950218` 기반)
- **변경 목적**: 승인 대상 물체가 동일하게 유지됨에도 주변 검출의 변화(추가/소실/위치 변경)로 승인이 부당하게 거절되던 전체 장면 엄격 비교 로직을 '승인 대상 전용 비교'로 개선.
- **주요 검증 결과**:
  - 주변 검출 변동 관련 테스트 7개 추가 후 22개 관련 검사 모두 통과.
  - 전체 `make test-dev-monitor` 182개 통과 (옵트인 18개 제외).
  - ROS Domain 231 합성 테스트: 주변 검출 소실 시 승인 유지, 대상 이동 시 거절, 정확한 Goal 1회 전송 확인.
  - 과거 실패 사례 3건의 로그 재생 검증 결과 모두 정상 통과.
  - 웹 및 ROS 4개 패키지 빌드 정상 통과, 유휴 상태에서 Dev Monitor 재시작 후 백엔드 준비 상태 확인. (실물 구동 미실시)
- **산출물 및 로그 위치**:
  - `runtime/scene-revalidation-diagnosis-20261023/` (`provenance.json`, `target-only-replay.json`, `monitor-regression.xml`, `ros-approval-regression.xml`, `deployment.json`)

## 2026-10-23 수거 파지 목표 추가 삽입 20mm

- **검증 환경**: Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble (Commit: `3950218` 기반)
- **변경 목적**: 수거 공용 설정 `collection_motion.yaml`의 `grasp_approach_offset_m`을 0.020m(20mm)로 적용하여 파지 성공률 향상 검증. (준비 위치 거리: 8/6/4cm → 보정 후 10/8/6cm)
- **주요 검증 결과**:
  - 집중 검사 11개 및 관련 회귀 검사 131개 전원 통과. 네이티브 및 모션 컨테이너 패키지 빌드 완료.
  - 실물 카메라 정면 캔 스냅샷(`rgbd-1792701273610493164-037980`) 대상 AnyGrasp 후보 58개 생성.
  - 왼팔: 후보 5번에서 4cm 준비 위치와 최종 목표 간 실제 거리 60mm로 두 구간 경로 계획 모두 통과.
  - 오른팔: 상위 32개 검사 실패. (실제 모터 이동 명령은 보내지 않음)
- **산출물 및 로그 위치**:
  - `../cleany-data/reports/grasp-insertion-20mm-20261023/` (`verification.json`, `implementation.patch`, `regression-tests.xml`, `selected-ik-diagnostics.json`)
  - 카메라 및 점군: `../cleany-data/reports/grasp-insertion-20mm-camera-20261023-053433/`

## 2026-10-23 캔 크기 추정 변동의 원인 분리

- **검증 환경**: Jetson Orin NX, ROS 2 Humble (Commit: `3950218` 기반)
- **진단 배경**: 음료 캔 승인 작업(`23d7ed83...`) 중 동일한 캔의 바닥면 단축 추정치가 9.3cm~13.4cm로 흔들려 11cm 상한 필터에 의해 탈락한 현상 원인 규명.
- **분석 결과**:
  - 약 10초 동안 20개 RGB-D 프레임 수집 및 진단 도구로 독립 분석 수행.
  - 마스크 IoU는 0.9942 이상, 평면 법선 변화는 0.224도 이하로 매우 안정적이었음.
  - 원인 규명: 캔 상단 경계면의 소수 포인트(0~15개)가 바닥 평면으로부터 5mm 이상 돌출되어 PCA 및 최소/최대 Bounding Box 계산에 혼입됨.
  - 검증: 진단용 깊이 분리(0.6m) 적용 시 실제 캔 폭인 5.27~5.57cm로 일관되게 수렴함을 확인.
- **산출물 및 재실행 안내**:
  - `runtime/footprint-diagnosis-20261023/` (`captures.npz`, `masks.npz`, `parameters.json`, `analysis.json`, `diagnostic.png`)
  - 재실행 절차: [실물 크기 추정 변동 진단](TESTING.md#실물-크기-추정-변동-진단-footprint-stability)

## 2026-10-23 Dev Monitor 작업 제안과 사용자 승인

- **검증 환경**: Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, Python 3.10.12 (Commit: `3950218` 기반)
- **검증 목적**: Mission Manager 없이 웹 UI에서 규칙 기반 추천, 사용자 단일 승인 및 1회 실행 제어 파이프라인 연동 검증.
- **주요 검증 결과**:
  - `make test-dev-monitor` 174개 통과, Playwright E2E(`operator.spec.ts`) 9개 전원 통과.
  - Domain 231 가상 테스트: 승인 전 대상 위치 재확인, 변동 시 거절, 승인 시 정확한 Goal 1회 전송 및 다음 제안 대기 검증 완료.
  - 실제 실물 인식 요청(`1d10ae5b...`) 약 7.49초에 완료, 캔(물체 ID 1)을 `READY` 상태로 정상 추천.
- **산출물 및 화면 캡처**:
  - `../cleany-data/reports/dev-monitor-proposals-20261023/` (`live-proposal-job.json`, `live-proposal.jpg`, `proposal-desktop.png`, `proposal-mobile.png`)

## 2026-10-23 접근 준비 자세의 4cm 대체 검사

- **검증 환경**: Jetson Orin NX, ROS 2 Humble (Commit: `3950218` 기반)
- **변경 목적**: 수거 접근 준비 거리 후보를 8cm, 6cm, 4cm 순서로 점진적 확장 (`pregrasp_fallback_offsets_m=[0.06, 0.04]`). 앞선 거리의 IK가 실패하거나 오차 초과 시에만 다음 거리 검사.
- **주요 검증 결과**:
  - Makefile 집중 검사 30개 및 회귀 검사 197개 통과. (순서 검사, 조기 중단, 4cm 충돌 거부 등)
  - 기존 성공 이력의 71개 후보 재검사 시 6cm 준비 자세로 경로 계획 정상 통과 확인.
  - 실물 팔 이동 명령은 보내지 않음.
- **산출물 및 로그 위치**:
  - `../cleany-data/reports/pregrasp-4cm-20261023/` 및 `pregrasp-4cm-camera-20261023-044405/` (`verification.json`, `fresh-ik-stats-*.json`)

## 2026-10-23 캔의 팔 선택 및 경로 계획 실패 수정

- **검증 환경**: Jetson Orin NX, ROS 2 Humble (Commit: `3950218` 기반)
- **수정 목적**: AnyGrasp 및 cuRobo IK 경로에서 8cm 준비 자세의 방향 오차가 30도를 초과(33.5도)하여 양팔 모두 실패하던 현상 수정.
- **해결 및 검증**:
  - 수거 프로필에 `pregrasp_fallback_offset_m=0.06`을 추가하여 6cm 대체 자세를 탐색하도록 보완.
  - 왼팔의 두 구간 경로 계획 성공 확인 (8cm는 31.5도로 탈락, 6cm는 27.7도로 허용치 이내 통과).
  - 집중 회귀 12개 및 관련 회귀 179개 전원 통과.
- **산출물 및 로그 위치**:
  - `../cleany-data/reports/arm-selection-fix-20261023-041756/` 및 `arm-selection-verification-20261023-042645/` (`comparison-summary.json`, `comparison.png`, `report.html`)

## 2026-10-23 DDS 수신 버퍼 대기 상태 비교

- **검증 환경**: Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, CycloneDDS 0.10.5
- **측정 내용**: 유휴 대기 상태에서 호스트 Best Effort 계측기의 수신 버퍼 크기를 208 KiB ~ 8 MiB까지 스윕하여 패킷 유실률 및 Hz 측정 (각 35초, 2회).
- **측정 결과 요약**:

| 요청 버퍼 크기 | 헤드 컬러 이동 구간 Hz 범위 | 계측기 UDP 패킷 유실 수 (2회 합계) |
|---|---:|---:|
| 208 KiB | 0 – 15 Hz | 23,789 |
| 512 KiB | 12 – 16 Hz | 936 |
| 1 MiB | 13 – 16 Hz | 30 |
| 2 MiB | 14 – 16 Hz | **0** |
| 4 MiB | 14 – 16 Hz | **0** |
| 8 MiB | 14 – 16 Hz | **0** |

- **결론**: 본 대기 상태 계측 환경에서 UDP 유실이 0으로 떨어지는 최소 요청 버퍼는 2 MiB로 나타남. (단, 운영 환경은 Reliable 수신 및 여유 마진을 위해 8 MiB 유지)
- **참고**: 재실행 절차는 [테스트 가이드](TESTING.md#dds-수신-버퍼-단계별-비교-dds-buffer-sweep)를 따릅니다. 도구 검사 3개 및 IO 기록기 검사 9개 통과.

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

- **검증 환경**: Jetson Orin NX (Ubuntu 22.04 arm64, ROS 2 Humble, Python 3.10.12, MoveIt 2.5.9, geometric_shapes 2.3.4, BT.CPP 4.10.0)
- **기준 커밋**: `d0afd48` (main) ──► `d81212e` (`integrate/jetson-main-curated`)
- **검증 목적**: `feat/jetson-manipulation` 브랜치 변경 사항 중 실물 조작 통합에 필요한 코드(364개 파일)를 선별하여 main 기반으로 체계적 통합 검증.
- **주요 검증 결과**:

| 검사 대상 | 통과 결과 | 원본 로그 파일 |
|---|---|---|
| 관련 네이티브 패키지 빌드 | 14개 패키지 전체 통과 | `build-final-3.log` |
| 인터페이스, 비전, 파지, 실물 상태, BT 회귀 | **3,150 passed**, 28 skipped | `package-tests-verified.log` |
| 호스트 CLI 및 유틸리티 도구 | **235 passed**, 3 skipped | `host-tools-tests-final.log` |
| 티칭 웨이포인트(TCP) 도구 | **36 passed**, 3 skipped | `tcp-tools-tests-final.log` |
| 가상 제어기(PTY) READ 전용 차단 및 쓰기 부재 | 2개 통과 | `controller-emulator-tests-2.log` |
| 로봇 모델 패리티, 캘리브레이션, 충돌 메시 | **74 passed** | `model-tests-final.log` |
| 씬 매핑, Feetech 드라이버, MoveIt 배치 C++ | 3개 패키지 통과 | `native-tests.log` |
| Dev Monitor 백엔드 (Python) | **154 passed**, 17 skipped | `monitor-tests-final.log` |
| Dev Monitor 웹 프론트엔드 (E2E) | **44 passed**, 3 skipped | `web-e2e-tests.log` |
| GPU Export 및 런타임 경계 가상 검사 | **34 passed** | `vision-tools-tests-final.log` |

- **산출물 위치**: `cleany-data/reviews/jetson-main-curation-20261021T173354Z/` (`verification.json`, `file-dispositions.tsv`, `source-preservation.json`)

## AnyGrasp 강제 GC 주기 비교 (2026-10-22)

- **검증 환경**: Jetson Orin NX 16GB (25W 모드, CUDA 12.6, PyTorch 2.8), Commit: `3950218`
- **변경 목적**: 추론 호출 시마다 발생하던 Python Full GC를 N회(`cuda_gc_interval_calls: 16`) 주기로 완화하여 파지 후보 생성 지연 시간 단축.
- **비교 측정 결과 (OCID 데이터셋 91개 유효 점군 대상)**:
  - **평균 소요 시간**: 2.292초 ──► **2.055초** (**10.3% 단축**)
  - **95 백분위수 (p95)**: 4.916초 ──► **4.554초** (**7.4% 단축**)
  - **후보 생성 일치율**: 91개 장면 전체에서 생성된 파지 후보 수 및 좌표 해시 완전 일치 (정확도 손실 없음)
  - **연속 256회 장기 실행**: 16회 간격 측정 시 프로세스 RSS 변화 약 +0.36 MiB로 안정적 메모리 유지 확인
- **산출물 위치**: `jetson-orin:/home/cleany/cleany-data/experiments/anygrasp-gc-20261022/` (`report.html`, `comparison.png`, `latency.png`, `summary.json`)

## 들어올리기 높이 보정 1회 전환 검증 (2026-10-22)

- **검증 환경**: Jetson Orin NX, ROS 2 Humble, Python 3.10, Motion 컨테이너, Commit: `3950218`
- **변경 목적**: 다회 재시도로 지연을 유발하던 파지 후 들어올리기 보정 로직을 '5mm 단위 올림 단 1회 추가 상승'으로 전환하여 시퀀스 안정화.
- **주요 검증 결과**:
  - 11mm 높이 부족분 발생 시 15mm 단일 추가 보정 실행 확인.
  - 관련 pytest **783개 전원 통과**.
  - `make build-manipulation-bt`(13개 패키지) 및 `make motion-container-prepare`(5개 패키지) 빌드 및 임포트 정상 통과.
- **산출물 위치**: `/home/cleany/cleany-data/reports/lift-single-correction-20261022/` (`pytest.log`, `pytest.xml`, `build.log`)

## AnyGrasp 입력, 사전 준비와 작업 유지 비교 (2026-10-22)

- **검증 환경**: Jetson Orin NX 16GB (25W), AnyGrasp SDK & cuRobo 0.7.8 컨테이너
- **측정 및 최적화 내용**:
  1. **점군 크기 및 타깃 ROI 비교**:
     - 고정 8,000점(2.054초) 대비 고정 4,000점(1.357초, 33.9% 단축), 가변 4,000/8,000점(1.740초, 15.3% 단축) 확인.
     - 형태가 뚜렷한 타깃은 4,000점으로 고속 처리하고 성긴 타깃은 8,000점을 유지하는 적응형 정책 채택.
  2. **워커 유지 (Lease) 효과**:
     - 매번 워커를 초기화할 때(9.848초) 대비 작업 Lease 유지 시 **1.603초**로 다음 파지 요청 응답 속도 대폭 개선. (상주 RSS 약 1.18 GiB)
  3. **VLM 사전 준비(Prepare Window) 연동**:
     - VLM 분석 시작 시 AnyGrasp 워커를 사전 로드하여 전체 파이프라인 대기 시간 단축 확인.
- **산출물 위치**: `jetson-orin:/home/cleany/cleany-data/reports/anygrasp-optimizations-20261022/` (`report.html`, `measurements.csv`, `summary.json`)

## manipulation-run AnyGrasp 후보 전환 (2026-10-22)

- **검증 환경**: Jetson Orin NX, ROS 2 Humble, Vision & Motion 컨테이너
- **변경 내용**: `make manipulation-run`의 기본 파지 후보 생성기를 AnyGrasp SDK 경로로 전환하고 `target_geometry` 적응형 점군 정책 적용.
- **주요 검증 결과**:
  - 도구 pytest **103개 통과**, Launch 검사 8개 통과.
  - 저장 픽스처(0002, 0001, 0002) 대상 `/grasp/plan` 호출 시 각각 29개, 21개, 29개 파지 후보 정상 반환 및 동일 워커 PID 유지 확인.
- **참고**: 현재(2026-10-23)는 리소스 절약을 위해 기본 모드가 네이티브 `geometric`으로 전환되었으며, AnyGrasp는 환경 변수(`CLEANY_COLLECTION_GRASP_PREDICTOR=anygrasp`)로 선택 가능합니다.

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

## 충돌 메시 평면화와 구 포함 범위 수정 (2026-10-23)

기준은 `7908ef2`의 `feat/jetson-manipulation-integration` 작업 트리다.
충돌 관련 최종 파일 hash는
[`source-hashes.json`](../output/collision-fix-20261023/source-hashes.json)에 기록했다.
Jetson의 `/tmp/cleany-collision-fix-20261023`와 별도 native/container build,
cache에서 검사했으며 운영 checkout을 수정하거나 실제 모터를 구동하지 않았다.
환경은 ROS 2 Humble, `cleany-motion:jp6.2`, cuRobo 0.7.8이다.

외접 메시의 팽창 상한을 원본 컴포넌트 볼록 외피 대비 1mm로 두고 지지 평면을
하나씩 추가해 작은 모서리를 단순화했다. 하완 오목 부품의 QEM과 양방향
표면 거리 0.25mm 기준은 유지했다. 대상 8종의 삼각형은 11,710→9,336,
양팔 GPU support 정점은 7,844→2,888이다. GPU 추가 외접 팽창 상한은 0.25mm다.
이 개수 변화는 실행 시간 개선율 측정이 아니다.

표면점 기반 구를 컴포넌트별 부피 포함 구로 교체하고, 링크당 16개로 병합했다.
현재 프로필의 구는 전체 120→240개다. 원본 CAD 정점과 메시별 표면 표본
928,625점에서 구 밖으로 빠진 점은 0개였다. base와 primitive는 이 구 검사
대상에 포함되지 않으며 전체 형상 검증에서는 유지된다. cuRobo 0.7.8은
자기 충돌 제약을 끄면 해당 비용도 함께 끄므로, 제안 구는 월드 장애물 회피에
사용한다. 후보와 경로의 자기 충돌 수락은 기존 MoveIt 또는 GPU 전체 형상
검사가 담당하며 SRDF와 live ACM은 변경하지 않았다.

최종 검사는 형상 core 28개, cuRobo/adapter 113개, 메시/MoveIt 17개,
CUDA 8개가 통과했다. 관련 native 3개와 motion overlay 5개 패키지 빌드도
통과했다. MoveIt 원본/경량 비교는 무작위 자세 128개와 근접 장애물 396개에서
기존 충돌 누락 0, 추가 자세 거부 0, 추가 접촉 2건이었다. 경량 모델의 경로
6개 상태는 원본 CAD에서도 유효했다. 별도 GPU 대조는 고정 바퀴, 머리 제외
모델의 동일 97개 자세와 native ACM을 저장해 CUDA에서 재생했다. 정규화한
URDF가 충돌 메시 경로 외에는 같음을 확인했고, MoveIt 허용 65개, GPU 허용
63개, GPU 충돌 누락 0개였다.

초기 8개 구의 단순 분할과 48개 구의 강제 자기 충돌 제약은 정상 IK를 거부해
채택하지 않았다. 기존 CUDA fixture의 continuous wheel 입력도 조작 프로필의
고정 바퀴로 수정했다. motion 이미지 내부 MoveIt 실행 의존성 부족과 직접
DDS 연결의 준비/응답 실패 때문에 최종 대조는 저장한 동일 입력을 재생했다.
대칭 구 병합의 플랫폼별 반올림 차이는 포함 관계를 검증한 뒤 원본 cache의
구를 표시하도록 해결했다. 초기 실패와 최종 XML, 로그는
[`결과 디렉터리`](../output/collision-fix-20261023/)에 보존했다.

[`수정 전후 그림`](../output/collision-fix-20261023/05-before-after.png),
[`3D 보기`](../output/collision-fix-20261023/collision-inspector.html),
[`검증 집계`](../output/collision-fix-20261023/verification.json)에서 결과를 확인할 수 있다.
실물 배포, 동작 확인과 처리 속도 벤치마크는 수행하지 않았다.

## 전체 충돌 메시 평면화 확대 (2026-10-24)

기준은 `7908ef29a1968088d458c11b586386450028c6d5`의 작업 트리다.
상완과 일부 모터에만 적용되었던 평면화를 하완, 어깨와 손목 연결부,
하완 모터, 머리와 마스트까지 확대했다. 17종의 자산, 전체 모델 28개 메시
인스턴스가 경량 자산을 사용한다. 이미 12–24개 삼각형인 집게 5종은 유지한다.
전체 38개 메시 인스턴스의 삼각형은 이전 적용 상태 70,600개에서 7,966개로,
하완은 8,128개에서 408개로 줄었다. 원본 CAD는 변경하지 않았다.

하완 포크와 오목한 연결부를 설정된 평면으로 자르고, 떨어진 조각을 따로
외접한다. 이전 하완의 8,000면 QEM 예외는 제거했다. 각 출력 조각은 입력
표면을 포함하며, 팽창 상한 1mm는 분할 조각의 볼록 외피 기준이다.
GPU의 추가 외접 팽창 상한도 1mm로 설정해 양팔 support 정점을
2,888개에서 1,766개로 줄였다. 이 수치는 실행 속도 측정 결과가 아니다.

Jetson의 격리된 `/tmp/cleany-collision-fix-20261023` checkout과 native/motion
overlay에서 검사했다. 환경은 ROS 2 Humble, `cleany-motion:jp6.2`,
cuRobo 0.7.8, Warp 1.6.2, Orin CUDA 12.8이다. native description/MoveIt
2개 패키지와 motion description/skill executor 2개 패키지 빌드가 통과했다.
최초 빌드는 underlay와 패키지 검색 범위 누락으로 실패했으며, native는 기존
underlay를 명시하고 motion은 이미지가 제공하는 5개 패키지로 검색 범위를
제한해 해결했다. 운영 checkout이나 실제 로봇은 변경하거나 구동하지 않았다.

형상 core 28개, 메시/MoveIt 30개, cuRobo/adapter 113개, CUDA 팔 검사 8개,
머리 포함 CUDA 비교 1개가 통과했다. MoveIt 비교는 무작위 자세 128개와
모든 22종 자산의 근접 장애물 684개에서 충돌 누락 0개, 추가 자세 거부 3개,
추가 접촉 3개였다. 생성된 경로의 6개 상태도 원본 CAD에서 통과했다.
머리 없는 GPU 비교는 이전에 저장한 동일 원본 CAD와 native ACM의 97개
상태를 재사용했다. 머리 포함 비교는 native MoveIt에서 129개 상태와 ACM을
새로 저장했다. 각각 충돌 누락 0개, 추가 거부 2개와 8개이며 기준 자세는
모두 통과했다. 정규화한 URDF가 충돌 메시 경로 외에는 같음을 검사했다.
구 240개는 원본 CAD 정점과 표면 표본 928,625점에서 누락 0개였다.

첫 후보의 통짜 볼록 외피는 원본 내부 면과 접촉하던 장애물 9개를 놓쳤다.
내부 면을 고려한 분할로 수정한 뒤 같은 장애물 전체를 다시 검사했다.
중간 하완 분할 후보는 포크를 분리하지 못해 단위 검사에서 거부했다.
QEM 후보의 표면 오차와 퇴화 삼각형 실패, 얇은 분할 조각의 외접 상한
실패도 보존했으며 이 후보들은 최종 자산에 포함하지 않았다.

원본 결과는 [`전체 검증 집계`](../output/collision-angular-20261024/verification.json),
[`파일 hash`](../output/collision-angular-20261024/source-hashes.json),
[`로그와 XML`](../output/collision-angular-20261024/),
[`평면화 전후 그림`](../output/collision-angular-20261024/06-angular-before-after.png),
[`전체 3D 및 부품별 그림`](../output/collision-angular-20261024/collision-inspector.html)에 있다.
형상 포함, 표본 검사와 실물 검증은 구분하며, 배포와 실물 동작은 수행하지 않았다.

## cuRobo 구 재검토와 형상 직접 맞춤 비교 (2026-10-24)

`feat/jetson-manipulation-integration`의 `7908ef29a1968088d458c11b586386450028c6d5`
위 미커밋 평면화 자산과 `collision-angular-20261024/source`의 URDF 및 검증된
cuRobo cache를 입력으로 사용했다. 로컬 NumPy/SciPy로 링크별 볼록 조각을 분할하고
구 중심을 최적화한 뒤, 모든 분할 정점까지의 거리로 외접 반경을 다시 계산했다.
기존 구와 비교안은 모두 15개 메시 링크에 16개씩, 총 240개다. base와 primitive는
제안용 구에서 제외되는 기존 범위를 유지했다.

| 항목 | 기존 병합 구 | 형상 직접 맞춤 16개/링크 |
|---|---:|---:|
| 하완의 방향별 CAD 외곽 초과 최댓값 | 23.9mm | 10.5mm |
| 오른쪽 가동 집게의 방향별 CAD 외곽 초과 최댓값 | 17.7mm | 8.1mm |
| 마스트의 방향별 CAD 외곽 초과 최댓값 | 59.9mm | 37.9mm |
| 선별한 4mm 장애물 4개에 대한 cuRobo 충돌 오탐 | 4 | 0 |

외곽 차이는 648개 방향의 표본값이며 모든 표면 간 거리의 상한이 아니다.
원본 CAD 정점과 면 표본 2,242,289개에서 비교안의 누락은 0이었다. 분할 전후
부피 보존과 분할 정점의 구 포함을 별도로 검사했다. 좌우 가동 집게는 공통
정규화 형상으로 맞춰 동일한 반지름 배열을 사용했다. CUDA 검사는 Jetson Orin의
`cleany-motion:jp6.2`, cuRobo 0.7.8에서 실행했다. 같은 장애물은 원본 CAD와
최적화 MoveIt 메시에서 모두 비충돌로 확인된 이전 검토 사례다.

첫 부피 중심 분할안은 큰 외곽 오차를 줄이지 못해 비교안에서 제외했다. 분할
후보 중 수치적으로 퇴화한 14개는 거부했다. 구 중심 최적화 12,457회 중 2회는
수렴 상태를 반환하지 않아 유한한 후보 중심 중 외접 반경이 작은 것을 선택하고
포함 반경을 재계산했다. 이 경로도 동일한 정점 포함 검사를 통과했다.

원본 수치, 소스 hash와 환경은
[`비교 결과`](../output/curobo-direct-fit-20261024/comparison.json),
[`분할 정점 증명 자료`](../output/curobo-direct-fit-20261024/certificates.npz),
[`CUDA 검사`](../output/curobo-direct-fit-20261024/curobo-witnesses.json),
[`비교 그림`](../output/curobo-direct-fit-20261024/sphere-fit-comparison.png)에 있다.
[`HTML`](../output/collision-angular-20261024/collision-inspector.html)은 기존 구와
16개, 32개 비교안을 선택해 표시한다. 이 비교안의 전역 최적성, 전체 계획
성공률과 속도는 검증하지 않았으며 로봇 설정 변경, 배포와 실물 동작은 하지 않았다.
