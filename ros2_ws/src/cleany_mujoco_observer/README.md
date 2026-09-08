# cleany_mujoco_observer

sorting의 `scheduled_cameras=true`에서는 원래 공통 카메라 renderer의 반복 촬영을
끄고 private snapshot 기반 센서 renderer를 사용한다. vendor의 0 Hz 설정에서도
발생하는 초기 1회 frame은 `/cleany/internal/disabled_vendor/*`로 격리한다.
공개 센서 토픽은 scheduled renderer만 발행한다. 물체 좌표/segmentation ID는
센서 토픽에 노출하지 않는다. `/sorting_cameras`의 `active_camera` ROS parameter를
`head`, `left`, `right`로 변경하면 **렌더링 자체의 주기**를 전환한다.
헤드 기본 10 Hz / 손목 사용 중 헤드 2 Hz / 선택 손목 10 Hz, 비선택 손목은 0 Hz다.
시작 파라미터는 `head_rate_hz`, `head_idle_rate_hz`, `wrist_rate_hz`이며
실행 중 rate 수정은 거부한다. `/camera/color/{image_raw,camera_info}` 및
`/camera/aligned_depth_to_color/image_raw`는 헤드 RGB-D이고,
`/{left,right}_wrist_camera/{image_raw,camera_info}`는 RGB 전용이다.
640×480, 촬영한 snapshot의 simulation timestamp, optical frame을 사용한다.
공개 손목 depth/GT mask는 발행하지 않는다. 이 모드는 GLFW offscreen context가 필요하다.
GLFW는 backend thread 시작 전에 동기 초기화한다. FPS는 simulation timestamp 기준
최대 목표치이며 렌더링 비용에 따라 낮아질 수 있다. 프레임 수 감소가 곧 전체 CPU/GPU
부하의 같은 비율 감소를 뜻하지 않는다.

MuJoCo 분리 수거 시뮬레이션의 결과를 독립적으로 평가하는 read-only 관측기다.
`cleany_mujoco_observer/ObservedMujocoSystem`은 설치된
`MujocoSystemInterface`를 상속하며 제어/physics 동작은 원래
backend에 맡긴다. 공개 `get_model` / `get_data`가 제공하는 physics mutex로
보호된 **별도 소유 snapshot**에서 동적 study-cafe
물체 collision geometry의 base-frame AABB를 읽어 `/simulation/sorting_ground_truth`에
`visualization_msgs/MarkerArray`로 발행한다. `text`는 body 이름이며
`pose.position`은 `base_link` 기준 AABB 중심, `scale`은 full extents다.
orientation은 단위 quaternion이다. 복사는 wall time 기준 최대 10 Hz이며,
mesh는 변환된 실제 vertex 극값, sphere/capsule/cylinder/ellipsoid는 해석적
투영 반경을 사용한다. 로컬 AABB를 회전시켜 다시 감싸면 기울어진 컵의 빈
공간까지 포함해 책상 아래로 경계가 내려가는 오류가 있어 이를 피한다.
이 변경은 평가 경계 계산만 수정하며 검출/제어 또는 배치 오차 허용값은 바꾸지 않는다.
발행도 simulation time 기준 최대 10 Hz다. full snapshot 복사에 따른 잠금
대기/제어 주기 비용은 실측 대상이며 hard-real-time 보장을 하지 않는다.

이 토픽은 검출/분류/grasp 생성/경로 계획 입력이 아니다. 시뮬레이션 테스트가
실제로 물체가 이동하고 올바른 수거함에 안착했는지 확인하는 oracle이다.
plugin은 qpos, ctrl, 힘, weld를 변경하지 않는다. 로봇 구동은 기존
ros2_control trajectory controller와 MuJoCo 접촉 물리 계산이 담당한다.

현재 설치된 Humble `mujoco_ros2_control` 0.0.3의 C++ API를 사용한다.
시스템 패키지를 수정하지 않는다. 기존 vendor read-loop plugin 경로는
physics의 `mj_copyData`와 동시에 arena/contact/force pointer를 읽을 수 있어
사용하지 않는다. 기존 `SortingObserver` vendor plugin 등록은 제거했다.
원래 backend의 동시 접근을 전반적으로 수정한 것은 아니며, 이 관측기의
GT와 contact만 일관된 private snapshot으로 보호한다. `sorting_observer`
publisher-only node가 launch parameter file을 초기화 시 읽으며 별도 spin
thread나 실행 중 parameter 변경 기능은 추가하지 않는다.

## 선택형 접촉 진단

분리 수거 launch에 `sorting_contact_diagnostics:=true`를 주면
`/simulation/contact_diagnostics` (`diagnostic_msgs/DiagnosticArray`)를 추가한다.
기본은 false이며, 켜진 경우에도 구독자가 있을 때만 최대 10 Hz로 만든다.
private snapshot에 이미 계산된 contact와 `mj_contactForce` 결과를 읽으며,
physics를 다시 계산하거나 힘을 적용하지 않는다.

header는 snapshot의 simulation time / `base_link`다. 첫 status는 전체/발행
접촉 수이며, 이후 status는 body/geom 이름과 id, base-frame 접촉 위치(m),
signed distance(m, 음수는 penetration), contact-frame normal force(N),
tangential force magnitude(N)를 key/value로 제공한다. normal force는 명령이나
그리퍼 파지 성공 판정이 아니다. 여러 접촉점의 힘을 하나의 물체 무게처럼
해석하지 않는다. 기본 최대 256개이며
`sorting_observer.maximum_contacts`로 1–4096 범위에서 설정한다.
상한 초과/geom id 없는 flex contact 생략 시 summary가 WARN이고 전체 수와
발행 수가 다르다. 이 진단은 controller/검출/분류/계획에서 구독하지 않는다.

```bash
make test-mujoco-observer
```

회전된 base-frame 위치 변환, 접촉 힘 조회, 읽기 전후 integration state 및
applied force 불변, 검사 상한/잘못된 snapshot 거부를 작은 MuJoCo 모델로
검사한다. 원본 arena 초기화 뒤에도 private copy의 접촉/힘이 유지되는지,
wrapper library가 hardware plugin으로 로드되는지도 검사한다.
실제 전체 분리 수거 성공을 입증하는 테스트는 아니다.

참고: [설치 버전 plugin/snapshot 처리](https://github.com/ros-controls/mujoco_ros2_control/blob/0.0.3/mujoco_ros2_control/src/mujoco_system_interface.cpp),
[MuJoCo contact force API](https://mujoco.readthedocs.io/en/3.4.0/APIreference/APIfunctions.html#mj-contactforce).
