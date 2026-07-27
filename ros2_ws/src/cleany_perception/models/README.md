# models/

Stable location for YOLO weights so the file path doesn't depend on where a
node was launched.

By default `detection_node` uses `weights: yolo11n-seg.pt` (detection +
instance masks), which ultralytics auto-downloads into the current working
directory on first run. Plain detect weights (e.g. `yolo11n.pt`) also work;
detections then carry no mask. To pin an exact file and a fixed location, put
the weights here and point the parameter at them:

```yaml
# config/detection.yaml
detection_node:
  ros__parameters:
    weights: <repo>/ros2_ws/src/cleany_perception/models/yolo11n-seg.pt
```

Weights are not checked in (see `.gitignore`). 배포 기준 모델은 자동 다운로드하지
않고 provenance에 기록된 URL과 SHA-256으로 받는다.

```bash
cd ros2_ws/src/cleany_perception/models
curl -fL \
  -o yolo11n-seg.pt \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-seg.pt
echo "55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152  yolo11n-seg.pt" \
  | sha256sum --check
```

`yolo11n-seg.pt.provenance`는 모델 파일명, SHA-256, 원본 URL, 라이선스
검토 상태를 기록한다. 다른 모델을 사용하려면 동일한 형식의 provenance를
코드 리뷰 대상으로 먼저 추가해야 한다.

Jetson Orin NX에서는 `.pt`를 직접 운영 경로로 사용하지 않는다. 대상 장치에서
`scripts/jetson/export-tensorrt.sh`로 FP16 `.engine`을 만들고 같은 실행
이미지에서 사용한다. export는 provenance를 먼저 검증하고 engine smoke test
후 `<engine>.manifest`를 생성한다. runtime은 engine SHA-256과 이미지 ID,
JetPack/TensorRT/CUDA/PyTorch/Ultralytics 정보를 모두 다시 검증한다.
TensorRT engine은 생성한 GPU와 런타임에 종속되므로 개발 PC나 다른 Orin에서
만든 engine을 복사하지 않는다.

Ultralytics 코드와 공식 모델은 AGPL-3.0 또는 별도 Enterprise 라이선스 조건을
따른다. 제품 배포 전에 적용할 라이선스를 사람 검토로 확정해야 한다.
