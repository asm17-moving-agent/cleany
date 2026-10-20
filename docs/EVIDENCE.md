# 실행 원본 색인

현행 사용법은 [실행](RUNNING.md), [테스트](TESTING.md), [연동](INTEGRATION.md) 가이드를 따릅니다. 실행 원본의 기본 데이터 루트는 `../cleany-data`이며 `CLEANY_DATA_ROOT`로 변경합니다. Clone에 포함되지 않은 외부 fixture, asset은 별도로 전달받고 hash를 확인합니다.

과거 pass 수, RTF, 결과는 기록 당시 버전의 근거입니다. 현재 버전의 검증 결과는 새 실행으로 확보합니다.

## Jetson 실물 마운트와 카메라 보정

2026-10-21에 `jetson-orin:/home/cleany/cleany`의
`52e44abe89b7103bd832f3cbf806a121c37cb82c`와 미커밋 작업 내용을 읽어 선별했다.
프로파일은 [jetson_20261009](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/)
아래에 보존한다. [provenance.json](../ros2_ws/src/cleany_description/config/calibration/jetson_20261009/provenance.json)은
보정 원본, 사용한 내부 보정, 과거 검증 보고서의 경로와 SHA-256을 기록한다.
검증 지표와 실패 또는 미검증 조건은 각 extrinsics YAML과 [캘리브레이션 가이드](CALIBRATION.md)에 있다.
보고서 원본과 이미지 데이터셋은 Jetson의 기록된 위치에 남아 있다.

[manipulation-detail.png](assets/physical-calibration/manipulation-detail.png)는
Jetson의 `cleany_description/mjcf/cleany.xml`과 기존 mesh로 MuJoCo 3.11.0에서
렌더링한 이미지다. `/tmp/cleany-shape-preview/manipulation-detail.png`에서
원본 그대로 복사했다. 카메라 시점과 관절 자세만 시각화용으로 설정했다.
양팔 shoulder yaw는 좌 -1.25, 우 +1.25 rad이고, shoulder pitch 0.45,
elbow pitch 1.1, wrist pitch -0.45, gripper 0.25 rad를 사용했다.
실물 제어와 calibration 재수행은 하지 않았다.

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
| `reports/documentation-reorganization-20261013/before/` | 이번 문서 개편 전 Markdown 사본 |
| `reports/documentation-reorganization-20261013/manifest.json` | 사본 source commit, dirty 상태, 각 파일 SHA-256 |

## 재현, 보존

문서 개편 전 사본의 source commit은 `fbe5854`입니다. Commit, dirty 상태, SHA-256은 보관 manifest, 과거 경로 대응은 `relocation-manifest.json`에서 확인합니다. 원본의 절대 경로와 hash는 보존하고, fixture의 현재 위치와 profile-manifest의 과거 provenance를 구분합니다.

재현할 때는 기록된 commit의 별도 checkout과 해당 환경, 입력을 사용합니다. 다른 브랜치의 자료는 해당 버전의 근거로 취급하며, 제품 결정은 KB 검토, Decision 절차를 따릅니다.
