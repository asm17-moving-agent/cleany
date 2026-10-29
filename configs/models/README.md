# 인식 모델 설정

실물 로봇의 2D 인스턴스 검출 및 세그멘테이션 마스크는 **YOLOE-26m-seg**를, 최종 물체 분류는 **Gemini**를 사용합니다.
모델 가중치(Binary)와 API 키는 보안 및 용량 관리를 위해 Git 저장소 외부에 보관합니다.
실행 파라미터는 `ros2_ws/src/cleany_perception/config/yoloe_real32.yaml` 및 Launch 파라미터에서 관리합니다.

## 1. 실물 로봇 기본 모델 경로

| 파일 설명 | 호스트 파일 경로 | 컨테이너 내부 마운트 경로 |
|---|---|---|
| **비교 및 복귀용 s 엔진** | `~/models/yoloe/real32/s/active/yoloe-26s-seg-real32.engine` | `/models/yoloe/real32/s/active/yoloe-26s-seg-real32.engine` |
| **기본 m 엔진** | `~/models/yoloe/real32/m/active/yoloe-26m-seg-real32.engine` | `/models/yoloe/real32/m/active/yoloe-26m-seg-real32.engine` |
| **s 원본 PyTorch 체크포인트** | `~/models/yoloe/yoloe-26s-seg.pt` | `/models/yoloe/yoloe-26s-seg.pt` |
| **m 원본 PyTorch 체크포인트** | `~/models/yoloe/yoloe-26m-seg.pt` | `/models/yoloe/yoloe-26m-seg.pt` |
| **텍스트 인코더** | `~/models/yoloe/mobileclip2_b.ts` | `/models/yoloe/mobileclip2_b.ts` |

- **환경 설정**:
  - 모델 디렉터리는 `/etc/cleany/jetson-identity.env`의 `YOLOE_MODEL_HOST_DIR`로 지정하여 읽기 전용(`ro`)으로 마운트합니다.
  - Gemini API 키는 환경 변수 `GEMINI_API_KEY`를 통해 프로세스에 주입합니다.
  - 실물 실행기의 모델 경로가 공유 프로파일보다 우선합니다. 기본 m 선택은 32종 프로파일과 export lock을 변경하지 않습니다.
- 원본 체크포인트는 [Ultralytics 공식 모델 표](https://docs.ultralytics.com/models/yoloe/#available-models)의 release 파일을 사용하며 export 전에 선택한 lock의 SHA-256을 확인합니다.
- **클래스 정의 (고정 32종)**:
  - 실물 환경에서는 `yoloe_real32.yaml`의 순서를 사용합니다. `phone`, `pen`을 대표명으로 사용하고 충전 어댑터는 `charger`에 포함합니다.
  - 물체 후보를 찾는 이름과 Gemini의 최종 종류명 및 처리 분류를 구분합니다. 대표명을 지정해도 통합한 세부 종류의 검출을 보장하지 않습니다.

## 2. 실물 32종 TensorRT FP16 빌드 및 검증

`yoloe-26s-seg-real32.lock.json`과 `yoloe-26m-seg-real32.lock.json`에 원본 체크포인트, 인코더, 동일한 프로파일의 SHA-256 해시와 환경 옵션이 고정되어 있습니다.

### 빌드 및 검증 단계
```bash
# 1. TensorRT Export 빌드 이미지 준비
make yoloe-trt-build

# 2. 32종 프롬프트를 고정한 엔진 내보내기
make yoloe-trt-export
make yoloe-trt-export YOLOE_TRT_ARGS='--model-size m'

# 3. 데이터셋 기반 엔진 검증 및 활성화
make yoloe-trt-validate YOLOE_TRT_ARGS='--dataset-root /absolute/path/to/datasets/ocid-cleany-100 --dataset-root /absolute/path/to/real32-camera-fixtures --activate'

# m 모델도 같은 입력으로 검증하며 모델 크기별 active를 별도로 관리
make yoloe-trt-validate YOLOE_TRT_ARGS='--model-size m --dataset-root /absolute/path/to/datasets/ocid-cleany-100 --dataset-root /absolute/path/to/real32-camera-fixtures --activate'
```

- **Export 산출물**: `~/models/yoloe/real32/<s 또는 m>/<생성시각>-<해시>/`에 PT, ONNX, Engine, `manifest.json`, `export.log`를 보존합니다. 검증 후 같은 크기의 `active`만 갱신합니다. 기본 실물 실행은 m을 사용합니다.
- **검증 데이터셋**: `--dataset-root` 아래의 `manifest.csv`를 사용하며, 100장 이상의 테스트 세트와 헤드/양손목 카메라 영상이 모두 포함되어야 합니다.
- `classes_present`는 32종 프로파일의 이름으로 작성합니다. 원본 34종 manifest는 보존하고 별도 manifest에서 이름을 대응시킵니다.

### 엔진 합격 기준
엔진 검증 시 원본 PyTorch 모델 대비 다음 기준을 모두 통과해야 합니다:
1. **검출 일치율**: Confidence ≥ 0.30 검출 기준, 동일 클래스 및 Bounding Box IoU ≥ 0.5 매칭률 **98% 이상**
2. **마스크 정확도**: 평균 Mask IoU **0.95 이상**
3. **추론 속도**: 중간값(p50) 처리 시간 **10% 이상 단축**, 95 백분위수(p95) 악화 없음
4. **활성화 (`--activate`)**: 위 기준을 모두 통과하면 `active` 심볼릭 링크가 새 엔진을 가리키도록 자동 갱신됩니다.

### 엔진 롤백 (복귀 절차)
이전 PyTorch 모델 또는 특정 버전으로 복귀해야 하는 경우 환경 변수를 지정하여 인식 컨테이너를 재시작합니다.

```bash
# 32종 s 엔진으로 복귀
YOLOE_MODEL_PATH=/models/yoloe/real32/s/active/yoloe-26s-seg-real32.engine make perception-run

# 현재 프로파일의 특정 검증 버전으로 실행
YOLOE_MODEL_PATH=/models/yoloe/real32/s/<버전>/yoloe-26s-seg-real32.pt make perception-run

# 또는 원본 PT + 로컬 인코더로 현재 32종 프롬프트 직접 구성
YOLOE_MODEL_PATH=/models/yoloe/yoloe-26s-seg.pt make perception-run
```

> [!TIP]
> **Gemini 모델 변경 시 엔진 재사용 안내**:
> 설정 파일에서 Gemini 분류기 모델 이름을 변경하더라도 전체 설정 파일 해시만 바뀔 뿐, 2D 검출기의 가중치나 TensorRT 그래프 구조는 변경되지 않습니다.
> 따라서 클래스 순서나 입력 해상도 등의 검출 설정이 동일하다면 기존에 검증된 TensorRT 엔진을 그대로 계속 사용할 수 있습니다.
