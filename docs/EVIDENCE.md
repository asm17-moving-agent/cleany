# 검증 범위와 결과

**작업 요청부터 현장 좌석 이동, 책상 정리, 복귀, 결과 보고까지 이어지는 전체 실물 미션은 검증이 남아 있습니다.** 아래 표는 각 환경에서 관찰한 결과와 그 결과가 뒷받침하는 범위를 요약합니다.

| 기능 | 시험 환경과 관찰 결과 | 검증 범위와 근거 |
|---|---|---|
| 센서와 로봇 모델 | 실물 Jetson에서 RGB-D 입력을 수신했습니다. 관절 상태의 시각에 맞춰 헤드·손목 카메라와 파지 프레임의 TF를 조회했습니다. | 모델과 센서 연결을 확인했습니다. [RViz·TF 측정 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L7-L19) |
| 장애물 지도 | Jetson에서 기록한 점군과 TF를 재생했습니다. OctoMap 갱신 최적화 전후에 모든 재생 단계의 지도 해시가 일치했습니다. | 같은 입력에 대한 지도 보존을 확인했습니다. 현장 주행 중 장애물 회피는 별도 시험입니다. [측정 조건과 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L280-L298) |
| 거리 센서 | 단독 ESP32 보드에서 ToF 센서의 8×8 거리 프레임을 120초간 수신했습니다. | 보드의 출력과 전송 연속성을 확인했습니다. 로봇 장착 상태의 감지 범위와 정지 거리는 현장에서 측정해야 합니다. [펌웨어](../esp32/tof_vl53l7ch/README.md#실물-기록과-재생) |
| 물체 인식 | 운영 환경에서 헤드 RGB-D 한 프레임을 입력해 객체 6개가 포함된 응답을 받았습니다. | 입력부터 응답까지 연결한 시험입니다. 검출률과 3D 위치 정확도는 별도 평가가 필요합니다. [인식 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L324-L336) |
| 단일 물체 수거 | 고정 베이스 MuJoCo에서 컵 하나의 수거, 놓기, 팔 복귀와 정지를 확인했습니다. 파지 중 취소와 제어기 중단 시의 결과도 검사했습니다. | 단일 물체 시뮬레이션의 결과입니다. 여러 물체의 수거 성공률과 실물 팔 성능은 별도 평가 대상입니다. [시뮬레이션 결과](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/ros2_ws/src/cleany_manipulation_bt/README.md?plain=1#L231-L306) |
| 미션 연동 | Gazebo/Nav2 주행과 Mock 조작 서버로 두 물체의 실행 ID, 재관찰과 미션 완료 보고를 확인했습니다. | 인식·계획·팔 동작에 모의 입력을 사용했습니다. [통합 시험 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md?plain=1#L1022-L1044) · [재현 절차](implementation/how-to/mission.md#gazebo-주행과-mock-조작) |

현장 인수검사에서는 좌석별 접근 좌표와 장착 보정값을 확인한 뒤, 한 작업 요청의 이동·수거·복귀·결과 보고를 같은 미션 ID로 추적합니다. [보정 가이드](implementation/calibration.md) · [실행 가이드](implementation/running.md)

패키지별 검사 명령과 입력은 [검증 가이드](implementation/testing.md)에서 확인할 수 있습니다. 표에 인용한 실험의 커밋, 환경, 입력과 원본 위치는 [전체 실행 기록](https://github.com/asm17-moving-agent/cleany/blob/11fbb1f6aeeaf6acd920927425f9a6141ad4d56f/docs/EVIDENCE.md)에 보존되어 있습니다. 변경별 자동 검사 결과는 [GitHub Actions](https://github.com/asm17-moving-agent/cleany/actions/workflows/ci.yml)에서 확인할 수 있습니다.

## Jetson 수거 검증 기록

아래 기록은 Jetson Orin의 ROS 2 Humble에서 수집했다. 운영 측정은 `f45801bc`의 현장 변경을,
왼팔 연속 복귀 검사는 그 작업본에 새 경유점과 사전 계산 궤적을 적용한 상태를 사용했다.
원본 경로는 Jetson의 `/home/cleany/cleany-data/runs/` 아래 상대 경로다.

| 항목 | 입력과 관찰 결과 | 원본 |
|---|---|---|
| 순차 준비 | 모의 준비 검사 50개와 가상 모터 검사 4개가 통과했다. 계획과 GPU 예열이 대기하거나 실패한 동안 토크 OFF가 유지됐다. | `torque-after-warmup-H8Bhb1LV/` |
| 손목 카메라와 기본 자세 | 손목 관측을 켠 구성과 끈 구성을 모의 시험했다. 웹 준비는 명령 제어기를 준비하고 복귀 자세를 확인했으며 팔 이동을 요청하지 않았다. | `disable-wrist-cameras-hjZwFow8/`, `web-prepare-current-pose-lsNRyTZv/` |
| 피드백 전송과 보호 | UDP 적용 후 읽기 전용 118.4초에서 5,846개 모터 표본의 최고 나이는 76.0 ms였다. 앞선 반복 시험에서는 ID 5 시리얼 읽기 시간이 초과됐다. 전송 전 실패만 재시도하는 조건은 PTY에서 확인했다. | `feedback-transport-0kcSsCC8/`, `feedback-prepare-20261118-0856/` |
| 그리퍼 가속도 | 양측 그리퍼가 raw 52를 50으로 제한했다. 설정을 50으로 맞춘 뒤 가상 시리얼 검사와 실물 준비 1회가 통과했다. 준비는 75.4초가 걸렸다. | `prepare-gripper-accel-20261120.Oi08YNnn/` |
| CAD 충돌 모델 | 같은 1 cm OctoMap과 관절 표본 138개에서 이전 모델과 간소화 모델의 유효 36개, 충돌 102개 및 접촉 링크 쌍이 일치했다. 측정된 시간 차이는 실행별 편차 범위에 있었다. | `cad-arm-shells-20261119.q4YWq6h0/`, `cad-share-deploy-20261120.u51DL5te/` |
| 파지 후보와 폭 | 기록된 PlanGrasp 요청 22건에서 후보가 생성된 경우는 수정 전후 각각 18건이었다. 안경 케이스의 중앙 윗면 폭은 새 방식에서 59.3 mm, 사람이 표시한 두 선의 역투영 폭은 63.5 mm였다. 배치 IK와 후보 제한은 저장된 OctoMap으로 격리 재생했다. | `grasp-top-width-20261120/`, `grasp-center-correction-20261120/`, `grasp-ik-shortlist-20261119.n5dfbr52/` |
| 원본 접근 궤적 | 시작점이 맞을 때 계획한 위치, 시간, 속도와 가속도를 사용했다. 시리얼 에뮬레이터는 보호 정지했고, 격리 GenericSystem의 JointTrajectoryController는 원본 86점 궤적을 받아 완료했다. | `native-rrt-live-deploy-20261123.UPDk1pQ8/`, `selected-trajectory-execution-20261123.0EXHpHyM/` |
| 왼팔 수거함 복귀 | 복귀 664표본과 마지막 구간 사전 검사 86표본에서 관절 한계, CAD와 저장된 깊이 지도 충돌 검사를 통과했다. 손목 roll 1틱 차이에서도 설치본이 궤적을 재타이밍했다. | `rear-bin-left-continuous-20261123.fi55_0ek/` |

이 기록의 가상 제어기와 저장 장면 재생은 해당 입력에서의 결과다. 새 복귀 궤적의 실물 실행 검증은 남아 있다.

## Jetson 추가 검증 원본

### 2026-11-23 준비 중 손목 카메라 서비스 확인 제거

개발 작업본 `707538de`와 Jetson 운영 작업본 `f45801bc`에서 왼쪽과 오른쪽 손목 카메라 노드를 시작하고 카메라 서비스 확인 없이 제어기 준비로 넘어가는지 모의 실행했다. 변경 전에는 첫 서비스 확인에서 실패했고 적용 후에는 제어기 준비에 도달했다. 전체 Jetson 셸 검사에서 기존 실패 11건은 같았고 통과는 88건에서 89건으로 늘었다. 실물 준비 명령은 실행하지 않았다. 원본은 Jetson `/home/cleany/cleany-data/runs/skip-wrist-prepare-AXnAUN9D/`의 `baseline.log`, `installed.log`, `jetson-before.log`와 `jetson-after.log`에 있다.

### 2026-11-19 Jetson MoveIt 체결 구멍 단순화 적용

Jetson Orin ROS 2 Humble에서 운영 기준 `f45801bc`의 CAD 충돌 메시 19개에 있는
지름 2.05~5.20 mm 체결 구멍 105개를 메웠다. 환경 충돌용 삼각형은 135,935개에서
126,185개로 감소했다. 각 메시의 좌표축별 외곽 범위는 같았다. 원본과 대체 GLB의
SHA-256 대응은 `configs/robot/cad_grasp.json`에 있다. 운영 제어 프로파일이 생성한
모델의 충돌 요소 245개를 대조했고 GPU 설치본의 모델과 대체 파일 19개 해시를 확인했다.

저장된 `grasp-ik-shortlist-20261119.n5dfbr52`의 OctoMap과 관절 상태에서 138개
자세 중 유효 36개, 충돌 102개였고 전후 접촉 바디 쌍도 일치했다. 격리 환경에서
기준 두 번과 운영 설치본의 검사당 CPU 시간 중앙값은 각각 12.08 ms와 11.26 ms,
서비스 응답 시간 중앙값은 15.05 ms와 14.33 ms였다. 실행별 측정 범위는 겹친다.
양손 그리퍼 전 범위 170개 자세, 장애물 투입과 제거, MoveIt 계획 요청이 통과했고
GPU IK는 양팔 각각 8개 목표에서 128개 해를 반환했다. 실물 구동 없이 검증했다.
원본, 입력 해시, 생성 스크립트, 형상별 결과, 교체 전 파일과 빌드 로그는 Jetson의
`/home/cleany/cleany-data/runs/cad-mount-holes-20261119.r8yo5CkR`에 있다.

### 2026-11-18 09:42 오른팔 파지 후보와 지지면 충돌

운영 Jetson의 기준 커밋은 `f45801bc`다. 승인 작업 `e4f2b1fa-37b1-449c-a4f0-a18bccd3e179`에서 양팔은 각각 15개 후보를 검사했다. 오른팔 후보 6, 9, 13은 자세 정렬을 통과하고 지지면과 손끝의 충돌로 탈락했다. 두 팔의 경로 계획 호출은 0회였다.

같은 실행의 관절 상태, URDF, 관측 물체 메시를 격리된 MoveIt에 넣어 세 후보의 접촉 링크를 재현했다. 지지면 윗면은 보존된 물체 메시 바닥을 사용했고, 바깥 범위는 관측 메시의 XY 폭과 운영 여유로 복원했다. 후보 6과 13은 양쪽 손끝, 9는 R 손끝이 지지면과 겹쳤다. CAD와 평면의 높이 차이는 접촉 서비스가 반환한 깊이와 별도로 계산했다. 10월 23일의 기존 그리퍼에서도 왼쪽 본체 및 가동 턱과 `perceived_support`의 충돌 판정 네 건을 확인했다.

09:42 원본 점군의 조회 기한이 지나 같은 track을 다시 촬영했다. 바운딩 박스 IoU는 0.998, 대표 위치 차이는 1.1 mm였다. 재촬영 점군으로 패드 접촉 위치를 조정한 후보가 실제 계획 서비스에서 오른팔 준비 및 접근 경로를 반환했다. 로봇에 궤적을 보내지 않았다. 09:52에는 관절 피드백 지연 260.4 ms로 제어기가 정지를 요청했다. 수정한 후보와 같은 위치, 방향, 개구폭의 기존 계획 성공 결과를 보존했다. 09:45에 저장한 OctoMap을 고정한 격리 재생에서는 변경 전후 모두 전체 경로를 선택하지 못했다. 당시 운영 지도는 센서 프레임마다 갱신됐으며, 실물 파지와 반복 성공 여부는 검증하지 못했다. 호스트 두 패키지, GPU 작업공간 다섯 패키지를 빌드했고 관련 Jetson 검사 254개가 통과했다.

원본은 Jetson의 `/home/cleany/cleany-data/runs/arm-path-20261118-0942/`에 있다. `contacts-replay.json`과 `right-gripper-support-0942.png`는 충돌 재현, `20261023-support-contacts.json`은 과거 기록, `recaptured.cdr`와 `choices4.log`는 재촬영 비교, `isolated-replay.log`는 저장한 1 cm OctoMap의 격리 재생 결과다.

### 2026-11-18 Jetson 실물 조작 화면의 로봇 준비와 카메라 TF

운영 작업공간 `/home/cleany/cleany`에서 현장 확인을 받고 dev-monitor의 실물 조작 화면을 통해 준비를 한 번 요청했다. 읽기 전용 시리얼 소유권을 해제하고 이전 부분 실행의 종료 및 모터 ID 1–14의 토크 OFF를 확인했다. 인증한 웹 화면에서 토크 확인 대화상자를 제출한 시각은 08:51:04 KST이며, HTTP 202 요청 ID는 `6cf67f5d-86b6-47bd-8622-74c6ff191d12`다.

`real_control` PID 1905366이 기동한 `move_group` PID 1905953에서 업데이터 SHA-256 `c17df8f18ade`를 확인했다. 임시 그리퍼 TF 발행기 없이 충돌 형상 245개를 등록했으며, 서로 다른 촬영 타임스탬프 3개에서 `integrated=1`이 기록됐다. 08:51:37에 모터 피드백 나이가 377.4ms에 이르러 보호 가드가 명령 컨트롤러를 정지했다. 준비 작업의 최종 결과는 `FAILED`다. 이후 관측 구간에서 관절 상태와 지도 반영 수신은 모두 0건이었다. 종료 시 ID 1–14 토크 OFF를 직접 확인했다. 일부 센서 지원 프로세스가 남아 관리 PID 기록은 유지됐다.

준비 실패 당시 RViz 원본은 `rviz/web-fault.png`에 있다. 토크 OFF 후 기존 읽기 전용 관절 서비스를 복원해 찍은 사진은 `rviz/read-only-tcp-cameras.png`다. 두 세션의 URDF SHA-256은 동일했다. 읽기 전용 실측 자세 14개 표본에서 양손 TCP의 로컬 −Y는 119.775mm의 공구 오프셋 및 140mm 접근 기준점 방향과 일치했다. 로컬 +X와 CAD 손끝 두 개를 잇는 직선의 각도 차이는 왼손 0.036°, 오른손 0.038°다. 헤드 및 양손목 광학 TF는 `camera_pose.json` 보정 모델과 수치 정밀도 범위에서 일치했다. 절대 장착 오차는 별도로 측정하지 않았다. 두 TCP와 세 카메라 좌표축을 표시하는 설정은 Jetson의 `~/.rviz2/cleany-physical-tcp-cameras.rviz`에 보관했다.

Jetson 원본 디렉터리는 `/home/cleany/cleany-data/runs/web-prepare-tcp-wrist-20261118-0849`다. `web-click.json`, `live/runtime.json`, `live/control-before-stop.log`, `live/web-job.jsonl`, `safe-stop.log`, `live/stationary-geometry.json`에서 요청, 프로세스, 실패, 토크 해제와 좌표축 검증을 확인할 수 있다.

### 2026-11-18 실물 읽기 전용 그리퍼 TF와 RViz

Jetson Orin의 `f45801bc` 작업공간에 읽기 전용 판독 노드와 launch 수정을 적용했다. 실물 관절 서비스만 재시작하고 dev-monitor가 사용한 물리용 MoveIt 설정으로 측정용 프로세스를 기동했다. 웹의 로봇 준비 버튼과 팔 구동은 이번 검사에 포함하지 않았다. ROS 도메인 0에서 이전 임시 그리퍼 TF 발행기 두 개는 꺼져 있었다.

30초 동안 단일 `/read_only_robot_state`가 모터 좌표 14개와 종속 관절 16개를 담은 메시지 600개를 발행했다. D435 점군 59개 중 36개가 지도에 반영됐다. 36개 모두 촬영 시각에 종속 링크 16개, 손끝 4개, 양손 TCP 및 카메라 프레임을 조회했다. 관절값은 보정 모델 해와 일치했고 TF 회전 오차는 최대 2.9e-16 rad였다. MoveIt 서비스에서 `base_link` 기준 해상도 1cm의 지도를 조회했다. 30초 검사가 끝난 뒤 촬영 시각의 충돌 형상 변환이 불완전해 한 점군을 건너뛰었고 후속 갱신이 재개됐다. 이 오류 로그는 해당 링크 이름을 제공하지 않는다.

관련 운영 패키지 검사 30개가 통과했다. Jetson 원본은 `/home/cleany/cleany-data/runs/read-only-gripper-tf-20261118-0840`의 `live/validation.json`, `live/move_group.log`, `live/scene.json`, `deployment.json`, `pytest-production.log`에 있다. `rviz/robot.png`, `rviz/grippers.png`와 `rviz/octomap.png`는 실물 ROS 토픽에 연결한 RViz 창을 직접 캡처한 것이다.

### 2026-11-18 Jetson dev-monitor 실물 OctoMap 운영 적용

Jetson Orin의 `/home/cleany/cleany` 운영 작업공간(`f45801bc` 및 검증한 소스 수정본)에서 dev-monitor가 선택한 설치 경로 `/home/cleany/cleany-data/builds/cleany/install`의 `cleany_scene_mapping` 패키지를 교체했다. 실행 중인 `move_group`의 코어와 업데이터 경로 및 SHA-256을 배포 전후에 확인했다. 업데이터 해시는 `c1edc3ac1f82`에서 앞선 동일 입력 재생과 패키지 검사를 통과한 `c17df8f18ade`로 바뀌었다.

운영 ROS 도메인 0에서 실물 D435와 읽기 전용 관절 피드백을 사용했다. dev-monitor가 07:43에 사용한 물리용 `move_group`의 실행 파일, ROS 파라미터, 충돌 검사기와 보정 모델을 그대로 사용해 측정용 프로세스를 각 버전에 한 번씩 기동했다. 궤적 실행은 꺼져 있었다. 첫 기동은 CAD 환경값 누락으로 종료됐고, 물리 launch와 동일한 환경값을 복원했다. 초기 읽기 전용 피드백에는 그리퍼 종속 관절 TF가 없어 기존본이 점군 처리를 거부했다. 두 버전 모두 운영 URDF의 보정 모델을 사용해 양쪽 그리퍼의 실측 각도에서 16개 종속 관절을 계산하고 같은 타임스탬프로 TF를 공급했다.

두 버전 모두 충돌 형상 245개, 1cm 지도, 2m 거리 제한, 2Hz 갱신 상한을 사용했다. 첫 반영 수신 후 15초 워밍업을 거쳐 40초씩 두 번 연속 측정했다. CPU는 전체 프로세스의 사용자 및 커널 CPU 시간으로 한 코어가 100%다. 처리 건수는 `/perception/scene_cloud_filtered`의 성공 수신 건수이며 지도 시간은 측정 구간에서 약 2초 간격으로 기록된 성공 로그의 `elapsed_ms`다.

| 지표 | 기존 운영본 | 최적화 운영본 | 변화 |
|---|---:|---:|---:|
| move_group 전체 CPU | 26.1% | 14.3% | 45.5% 감소 |
| 지도 반영 프레임당 프로세스 CPU | 209.2 ms | 111.9 ms | 46.5% 감소 |
| 지도 갱신 중앙값 | 81.0 ms | 34.2 ms | 57.8% 감소 |
| 지도 갱신 95백분위 | 98.7 ms | 45.8 ms | 53.6% 감소 |
| 광선 계산 중앙값 | 24.2 ms | 18.9 ms | 22.0% 감소 |
| 쓰기 잠금 안에서의 지도 처리 중앙값 | 44.7 ms | 5.0 ms | 88.8% 감소 |
| 검사한 리프 중앙값 | 44,323개 | 26개 | 99.9% 감소 |

80초간 입력은 160개와 160개, 지도 반영은 100개(1.25Hz)와 102개(1.27Hz)였다. 입력 점 수 중앙값은 7,183점과 7,188점, 카메라 좌표계 깊이 중앙값은 0.523m와 0.523m였다. 지도 시간 표본은 33개와 33개다. 마지막 40초 동안 dev-monitor 웹에 표시된 해당 PID의 CPU 중앙값은 24.4%(22표본)와 14.2%(22표본)였다. 촬영 타임스탬프를 맞춘 모든 성공 프레임에서 입력 점군 수신부터 갱신 결과 수신까지의 중앙값은 80.2ms에서 32.6ms로 줄었다. 두 구간에서 14개 실측 관절 각도가 동일했고 갱신 오류 로그는 없었다. 각 구간 종료 후 MoveIt 서비스에서 `base_link`의 1cm `OcTree`를 조회했다. 실물 구간은 서로 다른 시점의 점군이며, 동일 입력에서의 지도 동등성은 다음 절의 재생 결과에 기록했다.

측정 후 임시 MoveIt 프로세스와 그리퍼 TF 발행기를 종료하고, dev-monitor 웹, 실물 카메라와 읽기 전용 관절 서비스의 가동 상태 및 배포 파일 44개의 해시를 확인했다.

Jetson 원본 디렉터리는 `/home/cleany/cleany-data/runs/octomap-production-20261118-0834`다. `before/`와 `after/`에 CPU 구간, 점군 수신 기록, 웹 자원 표본, 원본 로그와 지도 서비스 응답 요약을 보관했다. `deployment.json`은 배포 전후 파일 해시와 기존 패키지 보관 위치를 기록한다.

### 2026-11-18 Jetson OctoMap 광선과 빈 공간 블록 쓰기

Jetson Orin, ROS 2 Humble에서 `f45801bc`와 충돌 형상 주변 리프 검사 수정본을 기준으로 광선과 빈 공간 쓰기를 비교했다. 기준본과 후보 모두 별도 설치 디렉터리에서 플러그인과 코어를 로드했다. 같은 1cm 해상도, 2m 거리 제한, 4개 마스크 워커, 충돌 형상 245개를 사용했다. 입력은 `octomap-ab-20261118-0759`에 보존한 점군과 TF다. 원거리 점군 3장(7,485점, 7,407점, 7,421점)과 근거리 점군 1장(7,177점)을 각각 빈 지도에 재생했고 원거리 3장은 촬영 순서대로도 재생했다. 근거리 대조군은 이전 재생에서 고정한 TF를 사용했다.

ROS 도메인 187에서 CPU 4–7에 고정하고 워밍업 한 쌍을 제외한 전후 10쌍의 실행 순서를 교대했다. 조건별 프레임 평균을 낸 뒤 구한 중앙값이며 단위는 ms다. 광선은 업데이터의 `rays_ms`, 지도 쓰기는 `tree_work_ms`에서 충돌 형상 리프 검사 시간을 뺀 값이다. 전체 CPU는 모든 스레드가 `processCloud`에서 소비한 시간이다. 초기화, DDS 전송, 지도 파일 저장은 측정 구간에 포함하지 않았다.

| 입력 | 광선 계산 | 지도 쓰기 | 전체 CPU | 갱신 경과 시간 |
|---|---:|---:|---:|---:|
| 원거리, 빈 지도 | 271.3 → 45.8 | 768.5 → 78.7 | 1,338.3 → 161.8 | 1,389.8 → 165.3 |
| 원거리, 누적 지도 | 279.7 → 44.1 | 973.7 → 93.6 | 1,487.7 → 145.5 | 1,512.2 → 146.9 |
| 근거리, 빈 지도 | 20.0 → 11.0 | 73.1 → 10.3 | 119.7 → 29.6 | 123.0 → 29.9 |

모든 재생 단계의 전체 OctoMap 직렬화 SHA-256이 기존 원본과 일치했다. 마지막 누적 지도는 502,460개 리프다. Ubuntu 설치 패키지 검사 56개가 통과했다. Jetson에서는 광선 및 블록 쓰기 검사 6개, 프레임 통합 검사 15개, 설치 플러그인 검사 1개가 통과했다. 수정 전후 재생에서 실패하거나 지도 해시가 달라진 회차는 없었다.

원본은 Jetson의 `/home/cleany/cleany-data/runs/octomap-block-update-20261118-0817`에 보관했다. `measurements.csv`에 회차별 CPU와 단계 시간, 지도 해시를 기록했다. `inputs-and-libraries.sha256`에는 점군, TF, 소스와 설치 라이브러리 해시가 있다. `block-update.patch`는 비교 기준본에 추가한 수정이다.

### 2026-11-18 Jetson OctoMap 충돌 형상 주변 리프 검사

Jetson Orin에서 운영 기준 `f45801bc`의 07:43 업데이터에 기존 코어와 수정한 코어를 각각 로드했다. 원본 입력은 `/home/cleany/cleany-data/runs/octomap-ab-20261118-0759`에 보관한 실측 점군 4장, TF, 충돌 형상 245개다. 근거리 대조군은 직전 재생과 같은 원거리 TF를 사용했다. 해상도 1cm, 최대 거리 2m, 검사 상한 200,000개다. 각 점군을 빈 지도에 적용하고, 원거리 점군 3장은 촬영 순서대로 누적 지도에도 적용했다. 수정 전후 전체 OctoMap 파일의 SHA-256이 모든 단계에서 일치했다.

실물 제어와 분리한 ROS 도메인에서 워밍업 후 실행 순서를 교대하며 10쌍을 측정했다. CPU 시간은 전체 `processCloud` 호출에서 모든 스레드가 사용한 프로세스 CPU 시간이고, 갱신 시간은 같은 구간의 경과 시간이다. 초기화, 지도 파일 저장, 운영 DDS 발행은 측정에 포함하지 않았다. 아래 값은 조건별 프레임 평균을 낸 뒤 구한 10회 중앙값이다.

| 입력 | 검사한 리프 전후 | 검사 CPU 시간 전후 (ms) | 전체 CPU 시간 전후 (ms) | 갱신 시간 전후 (ms) |
|---|---:|---:|---:|---:|
| 원거리, 빈 지도 | 200,000 → 26 | 26.91 → 4.41 | 1,349.18 → 1,322.24 | 1,383.51 → 1,340.59 |
| 원거리, 누적 지도 | 200,000 → 26 | 37.97 → 8.60 | 1,512.73 → 1,483.96 | 1,539.02 → 1,499.45 |
| 근거리, 빈 지도 | 18,131 → 25 | 6.67 → 1.94 | 124.97 → 116.05 | 128.88 → 119.64 |

Ubuntu Humble의 설치 패키지 검사 53개가 통과했다. Jetson에서도 충돌 형상 검사 14개, 설치 플러그인 검사 1개, 프레임 통합 검사 15개가 통과했다. Jetson의 별도 설치 경로에서 새 업데이터와 코어를 함께 로드한 첫 프레임은 07:43 기준 지도 파일과 바이트 단위로 일치했다. Jetson에서 기존 라이브러리가 먼저 로드된 첫 실행은 신규 검사 3개가 실패했다. 수정한 라이브러리의 경로를 고정하고 재검증했다. 성능 기록 한 쌍은 표준 출력과 ROS 로그가 섞여 해당 회차부터 분리 수집했다. 최종 원본은 Jetson의 `/home/cleany/cleany-data/runs/octomap-region-bench-20261118-0803`에 있으며, `verified-measurements.csv`에 회차별 측정값, `map-hashes.json`에 전체 지도 해시, `input-binary-sha256.txt`에 입력과 라이브러리 해시를 보관했다.

## 2026-11-25 Jetson 후방 배출 검사 통합

PR #113의 `9bad8eff` 위에 후속 워크트리 변경을 적용했다. Jetson Orin의
ROS 2 Humble 격리 도메인에서 모의 관절 피드백과 좌우 팔의 후방 배출 위치를 사용했다.
단독 수거는 물체가 테두리를 통과할 때 검사 후 열었고, 물체가 테두리 아래로 내려가면
물체를 부착한 상태에서 열기와 복귀를 중단했다. BT는 `CheckPlacementTarget`이 실패하면
`OpenGripperAtDestination`을 실행하지 않았다.

수거 경로와 계획 검사 273개, 새로 빌드한 BT 실행기를 사용한 검사 211개, 기하 파지 검사
43개, 운영자와 수거 셸 검사 143개가 통과했다. 실물 모터와 운영 서비스는 사용하지 않았다.
Jetson `/home/cleany/cleany-data/runs/jetson-runtime-followup-verify.efl7bd/`의
`rear-and-path.log`, `bt-build.log`, `bt-tests.log`, `grasping.log`, `operator.log`에
최종 실행 결과가 있다.

### 2026-11-25 Jetson 모터 및 드라이버 온도 한도 판독

Jetson Orin의 ROS 2 Humble 환경에서 기준 커밋 `98d33162`의 현장 설정으로 판독했다.
08:27에 토크가 해제된 모터 ID 1~14의 EEPROM 레지스터 13이 모두 80°C였다.
08:29에 기동한 드라이버 모델에서도 14개 관절의 온도 한도가 각각 80°C였다.
EEPROM 쓰기 건수는 0건이었고, 드라이버 준비 상태에 fault가 없었다.
원본은 Jetson의 `/home/cleany/cleany-data/runs/temperature-80-operating-profiles-20261125.q0ftr0yv/` 아래 `readback.json`과 `live-verified.json`이다.

## 2026-11-26 운영 Jetson UDP 수신 버퍼 8 MiB 적용

Jetson Orin NX 운영 작업본 `b38e68b5`의 ROS 2 Humble, Fast DDS 2.6.12에서
조작 실행기가 읽는 `fastdds_rgbd.xml`에 `receiveBufferSize=8388608`을 추가했다.
진행 중인 동작과 부착 물체가 없는 상태에서 조작 Action 서버만 재시작했다.
기동 검사가 Action 서버와 최신 보호 피드백을 확인했다. 커널이 표시하는 실행기의
DDS 수신 소켓 `SO_RCVBUF`는 16,777,216바이트였다.

동일 설정의 읽기 전용 서비스 묶음 32회에서 128건이 모두 응답했다. 최대 왕복은
`list_hardware_components` 17.6 ms, `list_controllers` 17.0 ms,
제어기 및 MoveIt `get_parameters` 각각 16.8 ms였다. 이 약 11초 동안 운영 실행기의
UDP 드롭 증가는 0건이었다. 재시작 전부터 Action 서버에 수거 실행 차단이 걸려 있었고
재시작 뒤 상태 조회에서도 확인했다. 모터 명령은 보내지 않았다.

Jetson 원본과 변경 전 프로파일은
`/home/cleany/cleany-data/runs/udp-buffer-deploy-20261126.NT2tplVb/`에 있다.

## 2026-11-26 제어기 목록 응답 재전송 원인 조사

Jetson Orin NX 운영 작업본 `b38e68b5`의 ROS 2 Humble, Fast DDS 2.6.12에서
조작 실행기와 같은 프로파일로 읽기 전용 조회 네 건을 함께 요청했다. 진단 클라이언트에서
DDS 응답 발행 및 수신 시각, UDP 패킷의 RTPS 헤더와 소켓 드롭 카운터를 기록했다.

03:38:41.701 KST의 `/controller_manager/list_controllers` 호출은 2,797.5 ms가
걸렸다. 요청 전송 직후 2.5 ms에 응답이 발행됐다. 응답을 받는 UDP 소켓의 드롭
카운터가 최초 발행을 포함하는 약 100 ms 계측 구간에 1 증가했다. 호출 후 2,783.7 ms에
DDS `HEARTBEAT`가 응답 시퀀스 285를 알렸고, 2,790.9 ms에 클라이언트가 `ACKNACK`을
보냈다. 같은 시퀀스의 재전송은 2,796.7 ms에 도착했다. `list_hardware_components`에서도
같은 흐름을 관측했다. Fast DDS의 기본 주기적 `HEARTBEAT` 간격은 3초다.
[Fast DDS WriterTimes](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/api_reference/rtps/attributes/WriterTimes.html).

제어기 프로파일은 UDP만 사용하고, 조작 실행기는 공유 메모리 및 UDP를 사용한다.
Jetson의 실행기 프로파일에는 UDP `receiveBufferSize`가 빠져 있었다. 위 호출과
같이 요청한 제어기 `robot_description` 응답의 UDP 데이터그램 네 개는 합계
약 234 KiB였고, 같은 소켓에 약 0.24 ms 사이에 들어왔다. 03:25의 3.046초와
3.036초 검증은 당시 패킷 기록이 없어 개별 재전송을 대조할 수 없다.

비교에 사용한 별도 진단 클라이언트는 운영 프로파일과 `receiveBufferSize`만
8 MiB로 지정한 사본을 번갈아 사용했다. 커널 `SO_RCVBUF` 표시값은 기존
212,992바이트, 8 MiB 요청 시 16,777,216바이트였다. 작업공간의 RGB-D
프로파일에는 이미 8 MiB 요청이 들어 있다.

완료된 첫 비교는 조건마다 같은 요청 묶음을 32회 실행했다. 두 목록 조회
64건의 최대 왕복은 기본값 2,794.3 ms, 8 MiB 요청 시 17.0 ms였다. 진단
클라이언트의 UDP 드롭은 각각 1,428건과 0건이었다.

모터 명령, 운영 설정 변경과 제어기 재시작은 없었다. 계측 원본은 Jetson
`/home/cleany/cleany-data/runs/controller-rpc-cause-20261126.yfTDsl3z/` 아래
`live_udp/`와 `buffer_ab/`에 있다.

## 2026-11-26 접근 전 ROS 서비스별 응답과 중복 검증 조사

Jetson Orin NX 운영 작업본 `b38e68b5`의 ROS 2 Humble, controller_manager
2.54.0에서 파지 실행기가 사용하는 환경으로 읽기 전용 서비스 네 개를 20회
동시에 요청했다. 서비스 발견은 모두 0.513 ms 이내였다.

| 서비스 | 왕복 중앙값 | 최대 왕복 | 2.5초 초과 |
| --- | ---: | ---: | ---: |
| `/controller_manager/list_hardware_components` | 9.10 ms | 3.019초 | 2회 |
| `/controller_manager/list_controllers` | 9.80 ms | 2.879초 | 5회 |
| `/controller_manager/get_parameters` | 8.65 ms | 106 ms | 0회 |
| `/move_group/get_parameters` | 8.85 ms | 27 ms | 0회 |

한 묶음에서 두 목록 조회가 2.807초와 2.810초 지연되는 동안 파라미터 조회는
각각 4.5 ms와 2.5 ms였다. 두 목록 서비스는 controller_manager의 배타적 콜백
그룹과 서비스 잠금을 공유한다. 03:25 실물 로그의 두 검증에는 서비스별 시간이
없고, 읽기 재현은 서버 콜백, 잠금, 전송 각각의 내부 시간을 구분하지 않는다.
[ROS 2 Humble controller_manager 2.54.0 구현](https://github.com/ros-controls/ros2_control/blob/2.54.0/controller_manager/src/controller_manager.cpp#L500-L540).

격리 도메인 227의 사본에서 안정화된 실측 시작점과 저장 궤적의 시작점을 먼저
대조하는 후보를 검사했다. 기존 코드와 후보의 모의 시나리오 20개씩이 통과했다.
시작점 불일치 경로의 모델 조회 횟수는 두 번에서 한 번으로 줄었다. 실제 전송에
대한 최신 검증을 그대로 수행하고, 맞는 궤적의 조회 횟수도 한 번이었다.
운영 코드 변경이나 모터 명령은 없다. Jetson 원본은
`/home/cleany/cleany-data/runs/approach-rpc-breakdown-20261126.aey2m6p4/`에 있다.

## 2026-11-26 pregrasp 도착과 접근 명령 사이 지연

Jetson Orin NX의 ROS 2 Humble 운영 작업본 `b38e68b5`에서 단일 승인 기록의
`MoveToPregrasp` 성공 시각과 `ApproachObject`의 첫 궤적 전송 직전 시각을 대조했다.

| 실행 | pregrasp 완료 후 접근 명령까지 | 모델 및 제어기 검증 | Cartesian 계획 호출 합계 |
| --- | ---: | ---: | ---: |
| 03:25 `8e277c8e` | 6.49초 | 3.046초, 3.036초 | 0.077초 |
| 03:21 `3a18c27f` | 0.74초 | 0.044초, 0.022초 | 0.086초 |
| 03:01 `35117f15` | 3.47초 | 3.025초, 0.046초 | 0.067초 |

03:25의 저장 접근 경로는 시작 관절 편차 0.0352 rad가 허용치 0.030 rad를 넘었다.
실측에서 재계획한 경로로 넘어가며 검증을 한 번 더 실행했다. 당시 로그는 모델과
제어기의 읽기 전용 조회 네 건을 한 묶음으로 계측한다. 운영 작업이 진행 중이어서
개별 서비스 재계측을 생략했다. 코드와 로봇 상태를 변경하지 않았다.

Jetson `/var/opt/cleany/state/cleany/manipulation_jetson/executions.sqlite3`와
`/home/cleany/cleany-worktrees/jetson-rear-bin-checks/runtime/collection-operator/`의
`20261126-031907`, `20261126-025604` 실행 로그가 원본이다.

## 2026-11-26 dev-monitor 배포 후 로봇 준비 경로 복구

Jetson Orin NX의 ROS 2 Humble 운영 작업본 `b38e68b5`에서 03:13과 03:15에
로봇 준비가 `Motor range return must be confirmed before command-controller startup`로
중단됐다. 상주 워커 배포에 사용한 설치 환경에서 최근 제어 패키지 경로가 빠져
관절 범위 복귀 완료 파일 `torque-ready`를 기록하는 제어기가 로드되지 않았다.
실패한 준비는 토크를 활성화하기 전에 종료됐고 정리 후 모터 14개의 토크 OFF를
다시 읽어 확인했다.

직전 성공 시 사용한 ROS 설치 경로를 복원해 dev-monitor를 다시 설치했다. 모니터,
상주 워커와 준비 하위 프로세스가 같은 다섯 패키지를 찾는 것을 확인했다. 관절 범위
복귀와 준비 검사 58개가 통과했다. 실물 준비 재실행과 모터 명령은 없었다.
원본은 Jetson `/home/cleany/cleany-worktrees/jetson-rear-bin-checks/runtime/collection-operator/20261126-031526/logs/`와
`/home/cleany/cleany-data/runs/prepare-overlay-20261126.KC10DsRN/`에 있다.

## 2026-11-26 dev-monitor 승인 작업의 ROS 연결 재사용

Jetson Orin NX의 ROS 2 Humble 운영 작업본 `b38e68b5`에서 dev-monitor 상주 작업 서비스를 적용했다.
단일 물체 승인은 상주 ROS Action 클라이언트를 통해 전달하고 실행기에서 확정 결과를 조회한다.
격리 localhost 도메인 223에서 동일한 단일 물체와 모의 Action 서버로 기존 경로와
새 설치본을 번갈아 각 5회 실행했다.

| 구간 | 기존 중앙값 | 변경 후 중앙값 |
| --- | ---: | ---: |
| HTTP 승인 요청부터 ROS Action 접수 | 5.113초 | 0.047초 |
| 작업 등록부터 ROS Action 접수 | 5.104초 | 0.032초 |
| 작업 등록부터 작업 시작 | 2.839초 | 0.006초 |

동일 요청을 다시 보내도 Goal은 한 번만 접수됐다. 승인 중 취소, Goal 거절과
확정 실행 기록이 없을 때의 `UNKNOWN` 처리를 확인했다. 설치 패키지로 자동 검사
177개가 통과했고 23개가 제외됐다. 운영 전환 뒤 모니터의 실행 파일과 HTTP 응답,
상주 워커 소켓을 확인했다. 실물 실행기가 종료된 상태였고 이동 명령은 0건이었다.

격리 도메인 225에서 03:00 실물 실행 `35117f15`의 로봇 모델, 관절 자세와
후보 개구값으로 cuRobo IK를 재생했다. 8개 목표와 16개 시드 구성에서 양팔 모두
예열 솔버 인스턴스를 재사용했다. 첫 호출은 바뀐 그리퍼 관절을 반영하는 전체
운동학 갱신이 각 1회 실행돼 왼팔 0.469초, 오른팔 0.464초가 걸렸다. 같은 입력을
반복하면 솔버 설정은 각각 약 1 ms였다. 이 구간은 솔버 설정 시간을 계측했다.

작업 전후의 비교와 원본은 Jetson
`/home/cleany/cleany-data/runs/approval-keepalive-20261126.wUZcH9bE/`의
`benchmark-final.json`, `curobo-reuse.json`, `pytest-final.log`, `deployment.json`에 있다.
