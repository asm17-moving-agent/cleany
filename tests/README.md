# tests

테스트 명령과 범위를 안내한다. 이 디렉터리에는 아직 실행 가능한 테스트가 없다.

ROS 2 패키지 내부 단위 테스트는 각 패키지의 `test/` 또는 `tests/`에 둔다.
같은 기능의 작은 검사는 관련 suite에 모으고 공통 준비 코드는 재사용한다. 소스 문자열이나
기본값을 반복 확인하는 검사보다 입력에 따른 결과·실패 처리·설정 전달을 검증한다.
실제 controller 실행과 접촉·충돌·취소·오래된 관측 차단 검사는 유지한다.

RGB-D grasp/pre-grasp 브랜치의 집중 검증은 레포지토리 루트에서 다음 두 target으로
실행한다.

```bash
make test-grasp-pregrasp
make test-grasp-pregrasp-runtime
```

첫 target은 perception, grasp·분류 core, 접촉 피드백, MoveIt 설정, MuJoCo 장면·
성능 프로필, URDF/MJCF 정합성과 C++ 충돌 지도·관측 검사를 실행한다.
두 번째 target은 OMPL/Pilz 계획과 가장 가까운 객체의 inspection 실패,
다음 객체 fallback, 열린 gripper 상태의 재검증과 MuJoCo pre-grasp controller 실행을
한 통합 테스트로 확인한다. 전체 workspace 회귀 검증은 병합 전 `make test`에서 별도로
수행한다. `make test`에는 vision container 계약 검사도 포함된다. Python 프레임워크는
pytest로 고정하며 사용자 환경의 관계없는 플러그인은 자동 로드하지 않는다.
카메라 runtime에는 headless 실행에서도 유효한 X display가 필요하다.

Jetson container의 identity, network, read-only mount fail-closed 정책은 SDK 없이 다음처럼
검사한다.

```bash
python3 -m pytest -q containers/vision/test
```

Feature ID, NVIDIA runtime, license와 실제 model warm-up은 Jetson 인수검사 항목이며
[`containers/vision` README](../containers/vision/README.md)를 따른다.
