# robot configs

로봇, 센서, frame, workspace, safety limit 관련 설정을 둔다.

## 메카넘 base

- `base_hardware.yaml`: 사용자 확인 geometry는 휠 직경 127 mm, 앞뒤 휠 중심 간
  350 mm, 좌우 휠 중심 간 610 mm다. 주행 제한은 `null`이며 안전 검토된 양의
  유한 값을 명시해야 실행할 수 있다.
- `base_synthetic.yaml`: 장치 없는 mock 테스트용 합성 profile. `mock:=true`에서만
  사용하며 실물 설정의 기본값으로 복사하지 않는다.

두 profile 원본은 `cleany_base_driver` package share에 설치된다. 같은 geometry를
driver 역기구학과 wheel odometry에 전달한다. 필드, launch와 검증 방법은
[`cleany_base_driver` README](../../ros2_ws/src/cleany_base_driver/README.md)를 따른다.
