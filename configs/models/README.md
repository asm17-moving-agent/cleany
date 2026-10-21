# 인식 모델 설정

현재 운영 인식은 YOLOE-26s-seg instance 검출·mask와 Gemini 분류를 사용한다.
모델 binary와 API key는 저장소 밖에 둔다. 실행 설정은
`ros2_ws/src/cleany_perception/config/yoloe_real34.yaml`과 launch parameter에서 관리한다.

## 현재 실물 기본 모델

| 파일 | 호스트 경로 | 컨테이너 경로 |
|---|---|---|
| 검증된 기본 engine | `~/models/yoloe/real34/active/yoloe-26s-seg-real34.engine` | `/models/yoloe/real34/active/yoloe-26s-seg-real34.engine` |
| 원본 checkpoint (변환·복귀) | `~/models/yoloe/yoloe-26s-seg.pt` | `/models/yoloe/yoloe-26s-seg.pt` |
| text encoder | `~/models/yoloe/mobileclip2_b.ts` | `/models/yoloe/mobileclip2_b.ts` |

모델 폴더는 root 소유 `/etc/cleany/jetson-identity.env`의 `YOLOE_MODEL_HOST_DIR`로
지정하고 read-only로 mount한다. 실물은 고정 34종을 사용한다. 클래스 ID와 순서의
단일 원본은 `yoloe_real34.yaml`이며 export와 runtime이 함께 읽는다.
컵은 종이컵·플라스틱컵·머그컵, crumpled paper는 구겨진 종이·휴지,
personal bag은 백팩·핸드백·토트백을 포함한다. `YOLOE_MODEL_PATH`와
`YOLOE_TEXT_ENCODER_DIRECTORY`는 컨테이너 내부 경로를 받는다.
Gemini API key는 `GEMINI_API_KEY` 환경변수로 공급한다.

## 실물 34종 TensorRT FP16

`yoloe-26s-seg-real34.lock.json`은 원본 PT·encoder·프로필 SHA-256과 환경·export
옵션을 고정한다. `make yoloe-trt-build`는 runtime 이미지에 export dependency만 더한
별도 이미지를 만든다. Jetson CUDA용 torch/torchvision wheel은 교체하지 않는다.

```bash
make yoloe-trt-build
make yoloe-trt-export
make yoloe-trt-validate YOLOE_TRT_ARGS='--dataset-root /absolute/path/to/datasets/ocid-cleany-100 --dataset-root /absolute/path/to/datasets/cleany-real34-saved-20261005 --activate'
```

기본 모델 폴더는 `~/models/yoloe`다. 다른 경로는 `YOLOE_TRT_ARGS='--model-dir ...'`로
지정한다. export마다 `real34/<UTC 생성 시각>-<lock hash>/`를 새로 만들고
34종 prompt를 저장한 PT, ONNX, engine, `input.lock.json`, `profile.yaml`, `manifest.json`,
`export.log`를 보존한다. 실패도 manifest·로그로 남기며 기존 binary를 덮어쓰지 않는다.
validate는 최신 완료 버전을 선택한다. `--version <절대 경로>`로 명시할 수도 있다.

검증은 실제 인식 adapter를 사용하며 640 square letterbox, `rect=False`, confidence 0.25,
최대 10개 검출, `retina_masks=True`로 원본 영상 좌표의 mask를 비교한다. 로딩·첫 요청을
분리하고 warmup 5회 뒤 전체 세트를 3회 반복한다. confidence ≥0.30인 PT 검출의
동일 클래스/bbox IoU ≥0.5 매칭률 98%, 평균 mask IoU 0.95, 처리 시간 p50 10% 감소,
p95 악화 없음이 모두 필요하다. 검출이 없어 비교 분모가 0이면 통과하지 않는다.
34종별 영상 확보 여부와 관측된 검출 수는 구분하여 보고한다. 이는 변환 회귀 기준이며
현장 정확도 인증이 아니다.

영상 입력은 각 `--dataset-root`의 `manifest.csv`다. `sample_id,rgb`는 필수이고
`camera`는 head/left_wrist/right_wrist, `classes_present`는 영상에서 확인한 클래스의
JSON 배열로 선택 기록한다. 라벨이 없는 클래스는 미검증이다. 평가 결과는 새
`validation-<시각>/` 아래 `comparison.json`, `raw.jsonl`, 전체 mask NPZ, 원문 로그에
남는다. Jetson 자원 계측은 별도 `telemetry-<시각>/`에 보존한다.
활성화에는 100장 세트와 저장된 head·양손목 영상이 모두 포함되어야 한다.

`--activate`는 모든 기준 통과 시에만 검증 자료의 hash를 manifest에 연결하고
`active` 상대 링크를 전환한다. perception container의 다음 시작부터 이 engine을
기본으로 선택한다. 엔진·manifest·lock·검증 hash, 클래스 순서, 입력 shape,
mask 계수/prototype binding이 다르면 시작 실패다. active 링크가 깨진 경우도 실패한다.
기존 active가 있으면 새 엔진은 그 엔진과 bbox 1px/confidence 0.01 엄격 동등성도
모든 영상에서 통과해야 한다. 같은 PT와의 엄격 비교는 초기 변환에서 진단으로만 기록한다.

복귀는 같은 34종 프로필을 유지하고 아래처럼 PT를 명시한 뒤 perception을 재시작한다.

```bash
YOLOE_MODEL_PATH=/models/yoloe/real34/<버전>/yoloe-26s-seg-real34.pt make perception-run
# 또는 원본 PT + 로컬 encoder로 같은 34종 prompt 구성
YOLOE_MODEL_PATH=/models/yoloe/yoloe-26s-seg.pt make perception-run
```

ROS launch의 `yoloe_config_file` 또는 container의 `YOLOE_CONFIG_FILE`로 프로필을
명시할 수 있다. 기존 4종 `yoloe_seg.yaml`/`yoloe_seg_gemini.yaml`과 스터디카페
학습 모델은 시뮬레이션 전용으로 계속 사용한다.

검증 보고서와 raw 측정값은 저장소 밖 데이터 루트에 보관한다. 모델 파일 유무와 실제 환경에서의 회귀 검증은 별도로 확인한다. 시뮬레이션 모델 준비는 [인식 reference](../../ros2_ws/src/cleany_perception/docs/runtime.md)를 따른다.

Jetson 이미지·workspace 준비와 실행은
[`containers/vision/README.md`](../../containers/vision/README.md)를 따른다.
