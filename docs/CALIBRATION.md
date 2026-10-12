# Hand-eye calibration 작업 가이드

현재 실행 경로는 MuJoCo wrist-camera/MoveIt 기반 eye-in-hand 수집이다.
결과 transform은 검토용 artifact이며 자동으로 TF에 적용하지 않는다.
실물 calibration·grasp 성공으로 해석하지 않는다.

## 작업 흐름

```mermaid
flowchart TB
    Setup["의존성 준비 + build / test"] --> Generate["새 profile · 후보 생성"]
    Generate --> Gate{"IK / collision / 렌더링 / PnP 통과?"}
    Gate -->|"예"| Freeze["20 calibration + 5 held-out 고정"]
    Gate -->|"아니오 / 후보 한도 초과"| Fail["실패와 근거 보존"]
    Freeze --> Collect["viewer 관찰 + settle 이후 수집"]
    Collect --> Data["image + journal + JSONL + hash"]
    Data --> Validate["PnP 재현 + solver별 평가"]
    Validate --> Review["dataset_validation.json 검토"]
    Review -. "별도 검토와 명시적 적용" .-> TF["실물 TF 적용"]
```

실선은 현재 MuJoCo 수집·평가 흐름이다. TF 적용은 자동 단계가 아니다.
실패 표본을 새 pose로 대체하지 않는다.

## 준비

[개발환경](DEVELOPMENT_SETUP.md)의 MoveIt·ros2_control·OpenCV 의존성을 준비한다.
좌표·camera·동기화·dataset 계약은
[frames-and-data](../ros2_ws/src/cleany_handeye_calibration/docs/frames-and-data.md)를 따른다.
[pose_generation.mujoco.yaml](../ros2_ws/src/cleany_handeye_calibration/config/pose_generation.mujoco.yaml)은
시뮬레이션 전용이다. `*.template.*`의 null/미승인 값을 실제 장치 값으로 사용하지 않는다.

```bash
source /opt/ros/humble/setup.bash
make build-handeye
make test-handeye
```

## 새 pose set 생성

저장소 루트에서 새 이름·출력 경로를 정한다. 기존 profile은 덮어쓰지 않는다.

```bash
export CLEANY_DATA_ROOT="$(dirname "$PWD")/cleany-data"
profile="$CLEANY_DATA_ROOT/fixtures/handeye/my-run"
artifacts="$CLEANY_DATA_ROOT/runs/handeye"
make handeye-generate-mujoco \
  HANDEYE_PROFILE_DIR="$profile" HANDEYE_ARTIFACT_ROOT="$artifacts" \
  HANDEYE_RUN_ID=my-run
```

후보 생성은 headless로 실행한다. 후보 cap을 넘기면 실패하며 runtime에서 새 pose로 빈 항목을 채우지 않는다.
`materialized_poses.yaml`, `materialized_runtime.json`, materialized URDF와 분석을 확인한다.

## 수집

```bash
make handeye-mujoco \
  HANDEYE_PROFILE_DIR="$profile" HANDEYE_ARTIFACT_ROOT="$artifacts" \
  HANDEYE_RUN_ID=my-run \
  HANDEYE_POSE_MANIFEST="$profile/materialized_poses.yaml" \
  HANDEYE_RUNTIME_CONFIG="$profile/materialized_runtime.json"
```

viewer/RViz를 열어 동작과 target visibility를 확인한다. Make target은 `headless:=false`다.
허용 motion·timeout·settle·collision 설정은 생성한 profile에서 검토한다.
joint feedback·camera 단절, PnP 실패, settle 실패를 무시하고 다음 표본으로 대체하지 않는다.
다른 arm/controller/backend를 동시에 실행하지 않는다.

## Dataset 확인

```bash
make handeye-validate-mujoco \
  HANDEYE_PROFILE_DIR="$profile" HANDEYE_ARTIFACT_ROOT="$artifacts" \
  HANDEYE_RUN_ID=my-run \
  HANDEYE_POSE_MANIFEST="$profile/materialized_poses.yaml" \
  HANDEYE_RUNTIME_CONFIG="$profile/materialized_runtime.json"
```

실제 robot-specific offset envelope를 검토한 경우 `HANDEYE_MAX_TRANSLATION_NORM_M`를 함께 지정한다.
유효성 상한을 결과에 맞춰 늘려 실패를 숨기지 않는다.

검토할 출력:

- run manifest의 code·환경·URDF/MJCF·profile·camera·board provenance
- 25개 row와 image, row/image/camera hash 및 calibration/held-out split
- 원본 ChArUco 대응점에서 재계산한 PnP와 solver별 유효성·실패 이유
- `dataset_validation.json`의 누락·불일치·평가 결과

표본 저장 완료와 solver 성능·물리 타당성은 구분한다. 한 solver의 실패를 다른 solver의 성공으로 지우지 않는다.
실물 적용은 frame 방향·장착 치수·offset·held-out 결과를 별도로 검토한 뒤 명시적으로 수행한다.

## 실패와 재현

부분 쓰기는 journal로 복구하며 이미 저장한 표본은 재수집으로 덮어쓰지 않는다.
재현에는 manifest가 기록한 동일 버전·profile·camera 계약이 필요하다.
container Git이 host worktree 형식을 지원하지 않으면 provenance 수집 단계부터 해결한다.
과거 실행 결과는 [근거 색인](EVIDENCE.md), 수학·저장 계약은 [reference](../ros2_ws/src/cleany_handeye_calibration/docs/frames-and-data.md)를 따른다.
