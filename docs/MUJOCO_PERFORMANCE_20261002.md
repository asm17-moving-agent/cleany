# MuJoCo 성능 측정 — 2026-10-02

PR #47의 `feat/mujoco-simulation-pr`를 `/home/ubuntu/Documents/cleany-mujoco`에
분리해 작업했다. 비교 기준은 `5922b148f695bfadb5004a73aaccc5dae748bd98`다.
아래 렌더링 전후 측정은 최적화 직후의 기록이다. 해당 측정에서는 원래 worktree,
canonical CAD mesh, 충돌·질량·마찰·solver·controller 허용치를 수정하지 않았다.
이후 마운트 정렬과 수거 중단 조건 수정은 아래 후속 통합 검증에 구분해 기록한다.

## 원인과 변경

이 Ubuntu 22.04 VM의 GLX는 VMware SVGA3D이며 `Accelerated: no`였다.
기존 renderer는 사용하지 않는 손목/별도 depth 카메라까지 공통 주기로 그렸고,
GUI는 모니터 크기로 별도 native renderer를 실행했다. 정지 상태의 physics는
1ms step당 약 0.227ms였으며, 먼 배경 충돌 602개를 제외해도 약 2% 차이였다.
이 실행에서 큰 병목은 RGB-D와 GUI 렌더링이었다. 충돌 mesh 단순화는 적용하지 않았다.

- 일반 인식에도 기존 수거용 카메라 scheduler를 사용한다. head RGB-D만 10Hz로
  촬영하며 손목이 선택되면 해당 카메라만 촬영한다. 별도 head depth 렌더는 생략한다.
- 촬영이 늦으면 원래 주기를 유지하며 놓친 기한을 건너뛰어 10Hz schedule drift를 줄인다.
- 기본 GUI는 센서와 OpenGL context/private snapshot을 공유한다. 960×720, 최대 20Hz,
  GUI 그림자 끄기를 기본으로 하며 물리 mutex 밖에서 그린다. 전체 UI는
  `sim_viewer:=native`로 선택한다. 센서의 640×480 해상도·그림자는 유지한다.
- observer C++ build의 미지정 최적화 수준을 `RelWithDebInfo`로 설정한다.
- 관찰용 Python bridge의 GUI sync도 최대 20Hz로 제한하고 GUI 그림자를 끈다.

## 비교 결과

같은 현재 CAD 장면, 기본 `baseline` 물리 프로필, 기본 simulation speed에서 측정했다.
인식 비교는 YOLOE plan-only, RViz/image view 끄기, clock 시작 후 warmup 5초,
30–40초 측정이다. GUI 변경 후 실제 960×720 창 표시를 X11 및 이미지로 확인했다.
초기 `efficient_viewer` 옵션의 대소문자 오류로 창이 열리지 않았던 측정은 제외했다.

| 측정 항목 | 변경 전 | 변경 후 |
|---|---:|---:|
| Headless head RGB-D 수신 빈도 | 6.87Hz | 10.00Hz |
| MuJoCo GUI + head RGB-D 수신 빈도 | 2.37Hz | 10.00Hz |
| GUI head RGB 프레임 간격 중앙값 | 429ms | 102ms |
| 인식 pipeline 실시간 진행률 | 약 1.0 | 약 1.0 |

MuJoCo·RViz·영상 뷰어를 모두 켠 기본 인식 launch에서도 RGB-D 10.00Hz와 진행률
0.99996을 확인했다. 전체 pipeline CPU 감소는 입증하지 않았다. 동일한 인식 비교의
전체 process tree CPU는 headless 약 5.12→5.40코어, MuJoCo GUI 약 5.28→5.57코어로,
개선 후 더 많은 센서 프레임을 처리했다. 위 수치는 이 VM의 단일 실행 비교다.

## 실제 수거 및 검증 범위

이 절은 렌더링 최적화 직후 실행의 결과다.
로컬 환경의 Gemini key를 사용해 headless와 실제 GUI에서 분류·손목 전환·파지·운반을
실행했다. 두 변경 후 실행 모두 레고를 놓은 뒤 `complete_unverified`로 진행했고,
다음 컵은 `Payload bounding sphere does not fit inside bin opening` 검사에서 멈췄다.
기본 수거 설정은 독립 배치 검증을 끄므로 레고의 수거함 내부 안착을 입증하지 않는다.
변경 전 실행도 레고 release 중 gripper trajectory 오차로 중단됐다.
**전체 수거 성공률 또는 총 수거 시간 단축은 입증하지 않았다.** 물리 수거 안정성은 남은 과제다.

완료한 관련 검증:

- `DISPLAY=:0 make test-grasp-pregrasp`: Python 811개 통과,
  scene mapping/observer colcon 결과 32개/24개, 실패·skip 없음.
- `cleany_mujoco_sim/test` 전체는 변경 작업 중 통과했다.
  LiDAR 원복 후 `test_state.py`: 22개 통과.
- 실제 OpenGL 검사: GUI draw 이후에도 headless와 RGB/depth 픽셀이 동일하며
  해상도·frame·동일 촬영 timestamp·CameraInfo를 확인했다.
- viewer sync 제한 중에도 모든 physics tick이 진행하는 테스트와
  xacro viewer boolean 소문자 출력 검사를 추가했다.
- 실제 GUI Space pause/resume 및 Backspace reset service 성공을 backend 로그로 확인했다.
  reset은 vendor 계약에 따라 물리 초기 상태를 복원하며 ROS 시각은 유지한다.
- runtime 측정기의 조기 종료/실패 및 자식 process 종료 경로를 확인했다.

마지막 GUI Escape/Home 재검증 때 실행 환경이 제한 모드로 바뀌어 DISPLAY 및 DDS
접근이 거부됐다. 이 키들의 실제 동작 검증은 완료하지 못했다. 관찰용 Python passive
viewer의 독립 측정 harness에서는 종료 시 GLX/segfault가 원본과 변경 후 모두 발생해,
이 측정의 정상 종료까지 입증하지 않는다. 측정 중 진행률·CPU 값과 실제 pipeline GUI
검증은 구분한다. ROS 전체 workspace의 무관한 navigation/Gazebo 테스트는 실행하지 않았다.

모든 테스트가 시작한 launch 및 추적한 자식 process는 종료했다. 최종 process 검사에서
관련 실행은 남아 있지 않았고, runtime JSON의 `remaining_test_processes`도 빈 배열이었다.

## 누적 개선 전후 레고 처리 시간

2026-09-11과 2026-10-02의 기존 `stages.jsonl`에서 레고 1개의 준비·인식·파지·운반·
놓기·복귀에 걸린 실제 경과시간을 비교했다. 시작은 `starting`, 끝은 과거 실행의
`verify` 진입 또는 최근 실행의 `complete_unverified`이며 독립 배치 검증 시간은 제외한다.

| 환경 | 9월 11일 | 10월 2일 | 감소율 |
|---|---:|---:|---:|
| GUI | 180.506초 | 63.022초 | 65.1% |
| Headless | 155.066초 | 65.040초 | 58.1% |

원본 stage 기록의 시작/종료 wall timestamp와 실행 ID:

| 실행 | run ID | 시작 | 종료 |
|---|---|---:|---:|
| 과거 GUI | `31648eb832ea487b9f616f9259cb018f` | 1789123204.367437 | 1789123384.8738916 |
| 최근 GUI | `c1c500fa5b0b4890af9e749efae101b2` | 1790886693.3416874 | 1790886756.3638127 |
| 과거 Headless | `fe54ed50f81145779d4c3f97ce0800d2` | 1789122446.5453076 | 1789122601.6114273 |
| 최근 Headless | `61c37905db254d92bfb9527454f0a8c9` | 1790886137.6594977 | 1790886202.6998286 |

과거 기록은 원래 worktree의 `artifacts/baseline_full_gui_20260911_latest/`와
`artifacts/tabletop_optimization_20260911/baseline_pipeline/`, 최근 기록은
`/tmp/cleany-perf/sorting-optimized-visible-gui/`와 `sorting-optimized-headless/`에 있다.
물리·렌더링 외에 인식·운반 로직 변경도 포함하며, 과거 GUI는 RViz를 켰고 최근은 껐다.
따라서 이는 누적 변경 이후의 관측 비교이며 렌더링 최적화 단독의 효과나 전체 네 물체
수거 완료 시간·성공률 개선으로 해석하지 않는다.

## 후속 통합 검증

마운트 중심 정렬, 알려진 메시의 OctoMap 잔상 제거, 수거함 입구 크기 검사 삭제와
손목 CHECK 미검출 시 그리퍼 피드백 보완을 반영했다. 손목 확인 3회 설정으로 실행한
`/tmp/cleany-wrist-recovery/runtime2.log`에서는 마우스 미검출 3회 후에도 새 위치·속도
피드백으로 파지 상태를 확인해 운반·놓기·복귀까지 진행했다. 레고·컵·마우스는 모두
`complete_unverified`였으며 독립적인 수거함 내부 안착 검증은 수행하지 않았다.
이후 휴지 pregrasp에서 오른팔 `PATH_TOLERANCE_VIOLATED`로 중단됐다.

최종 기본 설정은 카메라 CHECK 1회이며, 미검출 시 추가 촬영 없이 새 그리퍼 피드백
확인을 통과하면 계속 진행한다. 이 1회 설정은 단위 검사로 확인했으며 전체 수거를
다시 실행하지는 않았다. `DISPLAY=:0 make test-grasp-pregrasp` 최종 결과는 9개 패키지
빌드 성공, Python 859개 통과, scene mapping/observer colcon 결과 34개/24개로
오류·실패·건너뜀 0개였다.

## 재현

`GEMINI_API_KEY`는 로컬 환경에서 설정한다. 키를 기록/출력하지 않는다.

```bash
DISPLAY=:0 make profile-mujoco-runtime \
  BENCHMARK_ARGS='--duration 35 --warmup 5 --output /tmp/runtime-headless.json' \
  PIPELINE_ARGS='headless:=true use_rviz:=false use_image_view:=false'
DISPLAY=:0 make profile-mujoco-runtime \
  BENCHMARK_ARGS='--duration 35 --warmup 5 --output /tmp/runtime-gui.json' \
  PIPELINE_ARGS='headless:=false use_rviz:=false use_image_view:=false'
```

원본 비교에서는 기본 vendor renderer를 사용한다. 변경 후의
`scheduled_cameras:=false sim_viewer:=native`는 해당 renderer로 돌아가는 진단 옵션이다.
손목 전환을 사용하는 sorting에는 `scheduled_cameras:=false`를 사용하지 않는다.
상세 설정·조작·측정 도구 의존성은 각 패키지 README를 따른다.
원시 결과 요약은 [JSON](MUJOCO_PERFORMANCE_20261002.json), 실행 로그 및 stage artifact는
이 환경의 `/tmp/cleany-perf/`에 있다.
