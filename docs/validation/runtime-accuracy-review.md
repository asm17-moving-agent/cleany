# Runtime 위치 오차 및 구현 검토 (2026-10-08)

## 범위와 실행 환경

새 워크트리 `runtime-accuracy-review`, 브랜치 `fix/runtime-accuracy-review`에서
`feat/runtime-integration`의 `51ac1be`를 기준으로 작업했다. 기존 워크트리의
미커밋 변경은 가져오지 않았다. 이전 성능 개선 두 커밋을 이식한 기준은
`1745a58`이다. 원격 검증도 같은 별도 워크트리를 사용한다.

실행 장비는 `donglee@142spp-endeavour`의 Ryzen 5 9600X / RTX 5070이다.
Ubuntu 22.04 ROS Humble 컨테이너, Gazebo Fortress, NVIDIA Ogre2 EGL을 사용한다.
AMCL은 기존 검증 환경의 소스 변경 없는 Nav2 1.1.20 재빌드 overlay를 사용한다.

주행은 전체 시설에서 43번 좌석 왕복 후 13번 좌석 왕복이다. Navigation만
Gazebo이고 perception/planning/execution은 mock이다. 물리 로봇이나 실제 청소
성공을 증명하지 않는다. 요청 RTF 4, physics step 6.6667 ms,
`compact_static_scene:=true`, reactive safety와 obstacle memory를 유지한다.
이 step의 결과는 기본 1 ms 물리 해상도의 2배속 달성 근거가 아니다.

## 위치 오차 원인

Nav2의 SUCCESS는 localization으로 추정한 위치를 기준으로 판단한다.
기준 실행에서 추정 home 도착 오차는 약 5.5 cm였지만 Gazebo ground truth의
오차는 14.4 cm였다. 같은 순간 map→base 추정과 ground truth 차이는 약 9.0 cm다.
따라서 8 cm runtime 도착 게이트를 통과해도 실제 위치가 8 cm 안에 들어온다고
해석할 수 없다.

동일 시간의 wheel odometry를 초기 spawn 자세로 world에 변환하면 ground truth와
수 mm 수준으로 일치한다. 이동 중에도 차이가 약 1 mm인 구간에서 AMCL 위치는
약 8.8 cm 어긋났다. 이 fixture의 주된 오차는 wheel 기구학보다 localization이다.

동일 raw scan을 5 cm 지도 거리장에 재투영하면 ground-truth pose보다 y 방향으로
2~3 cm 이동한 위치의 likelihood가 높았다. 지도 양자화와 넓은 likelihood field가
작은 편향을 만드는 것을 확인했다. 이것만으로 전체 9 cm를 설명할 수는 없다.
기준의 alpha1..5=0.1을 이상적인 wheel odometry에 적용하면 불필요한 입자 분산을
추가한다. alpha만 낮춘 대조 실행과 sigma만 낮춘 대조 실행을 비교해, 현재
시뮬레이터에서는 motion noise 보정과 더 좁은 laser field를 함께 적용할 때
오차가 줄어드는 것을 확인했다. 실물 wheel slip/noise에 대한 결론은 아니다.

거리장 재투영은 Humble 1.1.20의 [likelihood 구현](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp)과
[지도 좌표 변환](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/include/nav2_amcl/map/map.hpp)을 따른다.

## 변경

- 선택 적용하는 `amcl_sim_ideal_wheels.yaml`: alpha1..5=0.001,
  sigma_hit=0.05. 추가 odometry noise가 없는 현재 simulation 전용이다.
  일반 기본 AMCL이나 실물 로봇의 noise를 바꾸지 않는다.
- `localization_params_file`은 AMCL parameter만 덮어쓸 수 있다. motion/safety
  overlay를 우회하는 설정은 거부한다.
- 공통 omni controller의 xy 도착 허용 오차는 5→3 cm로 더 엄격하게 했다.
  runtime의 원래 8 cm 도착 게이트와 yaw, freshness, speed, acceleration,
  deadline은 그대로 유지한다.
- mission debug/result에 도착 판단에 사용한 localization pose, frame, source,
  age와 오차를 남긴다. 값은 추정 위치이며 ground truth라고 표시하지 않는다.
- grasp의 TCP rotation에 반사 행렬을 거부하고 설정 배열을 복사해 외부 변경을
  차단한다. PointCloud2 datatype/count/point/row/data 길이를 검증하고,
  TF에서 latest로 해석되는 capture timestamp 0을 거부한다.

## 최종 주행 반복 검증

같은 소스/설정으로 새 simulation과 새 mission DB에서 세 번 연속 실행했다.
모든 좌석 방문 및 home 복귀가 SUCCESS이고 마지막 `/cmd_vel`과 ground-truth twist는
0이다. parent launch 세 개의 shutdown exit code는 매번 모두 0이다.

| 반복 | 43 왕복 RTF | 43 뒤 home GT 오차 | 13 왕복 RTF | 13 뒤 home GT 오차 |
| --- | --- | --- | --- | --- |
| 1 | 2.2130 | 2.99 cm | 2.1800 | 6.48 cm |
| 2 | 2.1890 | 2.40 cm | 2.1495 | 6.49 cm |
| 3 | 2.2110 | 3.14 cm | 2.1731 | 6.08 cm |

좌석에서 WORKING으로 전환된 첫 관측의 GT 오차는 43번 2.79~3.16 cm,
13번 0.92~1.03 cm다. 전환 관측은 약 0.2초 간격이며 action callback 시각과
완전히 동일한 sample이라고 간주하지 않는다. home 마지막 localization 편향은
13번 뒤에도 4.14~4.63 cm 남아 있다. 이 개선이 위치 오차를 없앴다는 뜻은 아니다.
마지막 scan age는 0.027~0.140 sim s, depth age는 0~0.067 sim s였다.

원래 180초 navigation timeout, 8 cm runtime 도착 기준, sensor freshness 및
velocity/acceleration guard를 완화하지 않았다. 원본 비교 결과는
`artifacts/accuracy-review/summary.json`과 각 실행의 trace/result에 있다.

## 전체 구현 검토

17개 ROS package와 firmware, 도구, CI를 입력 검증, 시간/좌표계, 실패 경계,
자원 비용, 재현 가능한 검증 중심으로 검토했다. 대용량 mesh/texture를 재생성하거나
KB를 수정하지 않았다. 아래의 변경 없음은 실물 인증이나 결함 부재를 뜻하지 않는다.

| 경계 | 확인 및 처리 | 검증 또는 남은 한계 |
| --- | --- | --- |
| navigation / gazebo_sim | depth self-filter, sparse obstacle projection, static model 비용 개선을 이식; localization 분리 보정 | 실제 시설 왕복 및 raw scan/TF/GT 비교; 1 ms에서 2배속은 미입증 |
| mission_manager / control_bridge | 원래 goal 도착 검증과 결과 evidence 보존; durable journal, recovery/cancel, replay/outbox 경계 확인 | core 및 ROS service/action 검증; 실제 외부 Gateway 전송은 별도 |
| base_driver / base_odometry / base_interfaces | wheel geometry와 stamp/sequence/finiteness/timeout, stopped fail-closed 경계 확인 | 전체 package 테스트 및 firmware host gate; MCU 실물 미검증 |
| interfaces | 새 ROS message를 추가하지 않고 기존 JSON debug/result를 확장 | 17-package build; transport round-trip |
| grasping | proper rotation/불변 설정, capture timestamp, malformed cloud 거부 개선 | selector와 ROS 입력 계약 테스트; 실제 AnyGrasp 모델/팔 성공 미검증 |
| handeye_calibration | bounded feedback, frame/PnP, SO(3), durable dataset 검토; 실제 runtime corner 부족 재현 | 충돌 거리의 독립 분리 하한 보완 및 카메라 fixture 개선; 기준 유지 |
| mujoco_sim | camera public contract와 capture stamp 확인, geometric visibility와 실제 검출 구분 | 새 fixture EGL 24 corners; 실제 runtime 4회 샘플 저장; 실물 보정 미검증 |
| skill_executor / moveit_config | infrastructure failure와 motion deadline, finite command 및 planning scene 경계 확인 | package 테스트; 실물 acceleration limit은 하드웨어 근거 필요 |
| description | 현재 CAD frame/FK, sensor transforms 및 모델 일치 확인 | 전체 build/model tests; asset 변경 없음 |
| telemetry | paused/repeated/reset sim clock에 대한 freshness 구분 확인 | package tests; 실제 센서/장치 검증 아님 |
| dev_monitor | bounded queue/preview, SQLite/CDR 기록, stale/replay UI 계약 확인 | TypeScript/Vite 성공, Playwright fixture 8 passed; live UI 2 skipped |
| bringup | map ID, backend, partition, launch 경계 확인 | package tests; 실물 bringup 미검증 |
| ESP32 firmware | replay/stale/timeout/disconnect wheel command gate 확인 | strict compiler host tests 6개 성공; flashing/실물 미수행 |
| tools / CI | 환경 의존과 실패 원인을 구분; artifact/검증 명령 점검 | 도구 테스트 21 passed; ROS 전체 결과는 아래 별도 표시 |

## 검증 상태

- 전체 workspace build: 17 packages 성공.
- mission/bridge: ROS opt-in 포함 57 passed, 3 external mock skipped.
- grasp package: 19 passed. localization merge/config: 24 passed.
- 웹 build 성공; Playwright fixture 8 passed, 2 live tests skipped.
- firmware host tests 6개 성공, 독립 도구 테스트 21 passed.
- 최초 전체 workspace Xvfb 테스트: 773 tests, 0 errors, 1 failure, 21 skipped.
  당시 failure는 hand-eye 실제 camera stream의 13-corner/3-quadrant 검출이었다.
  최초에는 선언된 aiohttp dependency 누락과 GLFW display 부재가 있었으며,
  컨테이너 dependency 설치 및 CI와 같은 Xvfb로 이를 분리했다.
- 최종 전체 Xvfb `make test`: 775 tests, 0 errors, 0 failures, 21 opt-in skipped.
  17-package build/test 및 후속 Gazebo profile tools 7 passed.
  원본은 `artifacts/accuracy-review/full-tests-certified-final.log`다.
- 최종 시설 주행 3회: 43/13 왕복 6개 모두 SUCCESS, GT home 오차 2.40~6.49 cm.
- hand-eye 실제 single-pose 런타임 4회 연속 통과: 샘플 저장, capture/joint stamp
  bracket, original corner/quadrant/PnP/settle/collision gate 검증.

원본 로그, `result.json`, `trace.jsonl`, `shutdown.json`, scan likelihood 재투영 및
카메라 진단은 양쪽 워크트리의 `artifacts/accuracy-review/`에 보관한다.

## 손눈 보정 검토에서 확인한 추가 결함

실제 capture의 검출 실패는 관절 목표와 실제 gravity-loaded 피드백의 작은 차이에서
일부 board quadrant가 사라지는 영상으로 재현했다. EGL nominal pose만으로 실제
camera stream의 성공을 주장할 수 없다. 검출/PnP/settle/충돌 기준을 유지한 채
production detector로 자세를 재선정했다. 새 nominal pose는 24 corners, PnP RMSE
0.046 px이며 대표 gravity-loaded perturbation 세 개에서도 19~24 corners와
네 사분면/PnP가 유지됐다. 측정 nominal clearance는 11.61 cm이며 기존 10 cm
collision gate를 그대로 사용한다. 이후 실제 runtime 4회가 샘플 저장까지 통과했다.

자세 탐색 중 MuJoCo 3.4의 `mj_geomDistance`가 떨어진 CAD mesh/box 쌍에 0을
반환하는 별도 문제를 재현했다. 같은 compiled geometry의 모든 꼭짓점과 box
support를 world에서 투영한 독립 하한은 0.4672 m와 0.5602 m였다. witness segment
길이는 0.5024 m와 0.6506 m였지만, 이 길이를 거리 대체값으로 사용하지 않는다.
관련 [upstream 재현 보고](https://github.com/google-deepmind/mujoco/issues/3383)도
있으나 이번 수정의 근거는 현재 모델의 독립 support 계산이다.

nonpositive 결과에만 unit 방향의 full support interval gap을 보완한다. 이는
Euclidean 거리의 보수적 하한이며 margin과 수치 오차를 뺀다. 실제 겹침과
unsupported shape는 0을 유지하므로 10 cm collision gate를 완화하지 않는다.
분리/회전/겹침/미지원 형상/margin과 실제 CAD regression 4 tests가 MuJoCo 3.4에서
통과했다. 접촉 margin이 음수일 때도 거리 증명을 키우지 않도록 보완했다.

Python 설치는 MuJoCo 3.12, Humble C++ vendor는 3.4였고, control-scene 비교에는
격리된 Python 3.4 wheel을 사용했다. 버전을 맞춘 nominal render는 기존 3.12와
같이 20 corners여서 버전 차이를 이번 corner 실패의 원인으로 확정하지 않는다.
Canonical mobile model은 더 최신 dcmotor가 필요하므로 전역 Python dependency를
3.4로 내려 다른 backend를 깨뜨리는 변경은 하지 않았다.

## 재현 명령과 소스 기준

패키지 launch의 optional profile은 다음처럼 적용한다. fixture map, sensor/body
profile, 목표 좌표와 runtime config는 기존 검증 fixture를 그대로 사용해야 한다.

```bash
ros2 launch cleany_gazebo_sim amcl_nav2_safety.launch.py \
  map:=/path/to/facility_integration.yaml \
  localization_params_file:=/path/to/cleany_navigation/config/amcl_sim_ideal_wheels.yaml

xvfb-run --auto-servernum \
  --server-args="-screen 0 1280x1024x24 +extension GLX" make test
```

정확한 원격 실행은 `artifacts/accuracy-review/run.py`와 fixture 경로를 사용했다.
주행 당시 HEAD는 `1745a58`였으며 navigation/mission/grasp source는 아래 커밋과
동일한 미커밋 작업 상태였다. 원격 이식 후 해당 source tree 일치를 확인했다.

- `521bedb`: 도착 localization evidence 보존
- `cd96a79`: grasp rotation/capture 입력 경계 개선
- `ad14827`: simulation localization 보정과 엄격한 controller 도착 기준
- `e10cf50`: 독립 convex clearance 하한과 안정적인 hand-eye camera fixture

본 검증은 로컬/원격의 해당 새 워크트리에만 해당한다. 기존 checkout이나 main의
배포, PR/CI 및 하드웨어 상태는 이 결과로 갱신됐다고 주장하지 않는다.
