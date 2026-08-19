# tests

패키지 경계를 넘는 통합 테스트와 end-to-end 테스트를 둔다.

ROS 2 패키지 내부 단위 테스트는 각 패키지의 `test/` 또는 `tests/`에 둔다.

Jetson container의 identity, network, read-only mount fail-closed 정책은 SDK 없이 다음처럼
검사한다.

```bash
python3 -m pytest -q containers/vision/test
```

Feature ID, NVIDIA runtime, license와 실제 model warm-up은 Jetson 인수검사 항목이며
[`containers/vision` README](../containers/vision/README.md)를 따른다.
