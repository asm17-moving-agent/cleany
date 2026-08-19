# tests

패키지 경계를 넘는 통합 테스트와 end-to-end 테스트를 둔다.

ROS 2 패키지 내부 단위 테스트는 각 패키지의 `test/` 또는 `tests/`에 둔다.

RGB-D grasp/pre-grasp 브랜치의 집중 검증은 레포지토리 루트에서 다음 두 target으로
실행한다.

```bash
make test-grasp-pregrasp
make test-grasp-pregrasp-runtime
```

첫 target은 perception, 거리순 객체 선택, grasp 후보와 MoveIt 선택에 관련된
unit/contract 테스트만 실행한다. 두 번째 target은 가장 가까운 객체의 inspection 실패,
다음 객체 fallback, 열린 gripper 상태의 재검증과 MuJoCo pre-grasp controller 실행을
한 통합 테스트로 확인한다. 전체 workspace 회귀 검증은 병합 전 `make test`에서 별도로
수행한다.

Jetson container의 identity, network, read-only mount fail-closed 정책은 SDK 없이 다음처럼
검사한다.

```bash
python3 -m pytest -q containers/vision/test
```

Feature ID, NVIDIA runtime, license와 실제 model warm-up은 Jetson 인수검사 항목이며
[`containers/vision` README](../containers/vision/README.md)를 따른다.
