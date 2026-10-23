# 인식 모델 설정

실물 로봇의 2D 인스턴스 검출 및 세그멘테이션 마스크는 **YOLOE-26s-seg**를, 최종 물체 분류는 **Gemini**를 사용합니다.
모델 가중치(Binary)와 API 키는 보안 및 용량 관리를 위해 Git 저장소 외부에 보관합니다.
실행 파라미터는 `ros2_ws/src/cleany_perception/config/yoloe_real34.yaml` 및 Launch 파라미터에서 관리합니다.

## 1. 실물 로봇 기본 모델 경로

| 파일 설명 | 호스트 파일 경로 | 컨테이너 내부 마운트 경로 |
|---|---|---|
| **검증 완료된 기본 엔진** | `~/models/yoloe/real34/active/yoloe-26s-seg-real34.engine` | `/models/yoloe/real34/active/yoloe-26s-seg-real34.engine` |
| **원본 PyTorch 체크포인트** | `~/models/yoloe/yoloe-26s-seg.pt` | `/models/yoloe/yoloe-26s-seg.pt` |
| **텍스트 인코더** | `~/models/yoloe/mobileclip2_b.ts` | `/models/yoloe/mobileclip2_b.ts` |

- **환경 설정**:
  - 모델 디렉터리는 `/etc/cleany/jetson-identity.env`의 `YOLOE_MODEL_HOST_DIR`로 지정하여 읽기 전용(`ro`)으로 마운트합니다.
  - Gemini API 키는 환경 변수 `GEMINI_API_KEY`를 통해 프로세스에 주입합니다.
- **클래스 정의 (고정 34종)**:
  - 실물 환경에서는 `yoloe_real34.yaml`에 정의된 34개 클래스 목록과 순서를 단일 기준으로 사용합니다.
  - 예: `cup`(종이컵·플라스틱컵·머그컵), `crumpled paper`(구겨진 종이·휴지), `personal bag`(백팩·핸드백·토트백) 등 대표 카테고리를 포함합니다.

## 2. 실물 34종 TensorRT FP16 빌드 및 검증

`yoloe-26s-seg-real34.lock.json` 파일에 원본 체크포인트, 인코더, 프로파일의 SHA-256 해시와 환경 옵션이 고정되어 있습니다.

### 빌드 및 검증 단계
```bash
# 1. TensorRT Export 빌드 이미지 준비
make yoloe-trt-build

# 2. 34종 프롬프트가 적용된 TensorRT FP16 엔진 내보내기 (Export)
make yoloe-trt-export

# 3. 데이터셋 기반 엔진 검증 및 활성화
make yoloe-trt-validate YOLOE_TRT_ARGS='--dataset-root /absolute/path/to/datasets/ocid-cleany-100 --dataset-root /absolute/path/to/datasets/cleany-real34-saved-20261005 --activate'
```

- **Export 산출물**: `~/models/yoloe/real34/<생성시각>-<해시>/` 디렉터리에 PT, ONNX, Engine, `manifest.json`, `export.log`가 보존되며 기존 바이너리를 임의로 덮어쓰지 않습니다.
- **검증 데이터셋**: `--dataset-root` 아래의 `manifest.csv`를 사용하며, 100장 이상의 테스트 세트와 헤드/양손목 카메라 영상이 모두 포함되어야 합니다.

### 엔진 합격 기준
엔진 검증 시 원본 PyTorch 모델 대비 다음 기준을 모두 통과해야 합니다:
1. **검출 일치율**: Confidence ≥ 0.30 검출 기준, 동일 클래스 및 Bounding Box IoU ≥ 0.5 매칭률 **98% 이상**
2. **마스크 정확도**: 평균 Mask IoU **0.95 이상**
3. **추론 속도**: 중간값(p50) 처리 시간 **10% 이상 단축**, 95 백분위수(p95) 악화 없음
4. **활성화 (`--activate`)**: 위 기준을 모두 통과하면 `active` 심볼릭 링크가 새 엔진을 가리키도록 자동 갱신됩니다.

### 엔진 롤백 (복귀 절차)
이전 PyTorch 모델 또는 특정 버전으로 복귀해야 하는 경우 환경 변수를 지정하여 인식 컨테이너를 재시작합니다.

```bash
# 특정 검증 버전의 PT 모델로 실행
YOLOE_MODEL_PATH=/models/yoloe/real34/<버전>/yoloe-26s-seg-real34.pt make perception-run

# 또는 원본 PT + 로컬 인코더로 34종 프롬프트 직접 구성
YOLOE_MODEL_PATH=/models/yoloe/yoloe-26s-seg.pt make perception-run
```

> [!TIP]
> **Gemini 모델 변경 시 엔진 재사용 안내**:
> 설정 파일에서 Gemini 분류기 모델 이름을 변경하더라도 전체 설정 파일 해시만 바뀔 뿐, 2D 검출기의 가중치나 TensorRT 그래프 구조는 변경되지 않습니다.
> 따라서 클래스 순서나 입력 해상도 등의 검출 설정이 동일하다면 기존에 검증된 TensorRT 엔진을 그대로 계속 사용할 수 있습니다.
