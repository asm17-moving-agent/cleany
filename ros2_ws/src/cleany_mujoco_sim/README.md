# cleany_mujoco_sim

수거 시뮬레이션은 `robot_top_bins.yaml`의
`simulation_ignore_mast_collision: true`를 기본으로 사용한다.
고정 기둥(`top_base_link`) 외형은 남기되 MuJoCo 접촉을 끄고, MoveIt에서도
기둥과 로봇 링크 간 충돌을 제외한다. 팔·책상·수거함·물체의 충돌은 유지한다.
이는 시뮬레이션 편의 설정이며 기둥을 통과하는 동작은 실물에서 안전하지 않다.
해당 값을 false로 바꾸면 기둥 충돌 검사를 복원한다.

기본 책상 테스트 배치는 로봇 기준 왼쪽에 레고·마우스(분실물), 오른쪽에
종이컵·휴지뭉치(쓰레기)를 둔다. `study_cafe_layout.yaml`의 현재 로봇 yaw에서는
desk X 음수가 로봇 왼쪽이다. 같은 쪽 팔에서 같은 쪽 수거함으로의 도달성을
시험하기 위한 배치이며, 실제 분류와 팔 선택은 기존 인식 파이프라인을 유지한다.

스터디카페는 `cleany_description/mjcf/cleany.xml`의 migrated CAD 모델을 사용한다.
CAD 전면 기둥과 책상의 초기 겹침을 피하도록 chair 기준 후퇴 거리를 0.22m로
설정한다(기존 0.18m에서 4cm 후퇴). 내부 후면 수거함은 chassis에 부착한다.
초기 keyframe의 position actuator 명령은 actuator 이름이 아니라 연결된
joint 이름으로 찾는다. 따라서 `head_tilt` / `head_tilt_joint`처럼 이름이
달라도 지정된 카메라 자세를 유지하며 0rad로 돌아가지 않는다.
책상·물체·후면 수거함 배치는 유지하며, 기존 접촉 pair는 이름이 유지된 양팔
jaw collision geom을 참조한다. `robot_top_bins_preview.launch.py`는 모델 배치
확인용이며 자동 집기·수거를 실행하지 않는다. 전체 수거 성공은 별도 검증 대상이다.

날짜별 CAD 이관·수거 실행 결과는 [실행 기록](../../../docs/SORTING_PIPELINE_TRIALS_20260909.md)을
따른다. 단위 테스트 통과는 전체 물리 수거 성공률을 나타내지 않는다.
`make test-mujoco`는 현재 장면과 공통 bridge의 전체 단위 테스트를 실행한다. Python bridge의 시뮬레이션 context와 step observer는 단일 정의로 관리한다.

## 로봇 후면 수거함 배치 미리보기

`ros2 launch cleany_mujoco_sim robot_top_bins_preview.launch.py`는 기존 책상
양끝 분류 영역/구분선을 넣지 않고, 로봇 뒤쪽에 청록 테두리 분실물함(로봇 왼쪽)과
주황 테두리 쓰레기함(오른쪽)을 배치한다. 본체는 저채도 네이비로 통일한다.
`config/robot_top_bins.yaml`에서 base_link 기준 위치와 크기를 변경한다.
기존 실행 호환을 위해 파일명은 유지하지만 현재 배치는 상단이 아닌 후면이다.
각 함의 중심은 base_link 기준 x=-0.075 m, y=±0.105 m이며
바닥 z=0.22 m, 상단 z=0.34 m이다. 외부 선반 대신 `internal_tray` 설정으로
210×460×12 mm 받침판을 로봇 프레임 내부 뒤쪽에 둔다.
함과 받침판은 chassis에 부착되어 로봇을 따라 움직이며 동일한 충돌 형상을
MoveIt에 등록한다. 함의 탈착·체결부 강도·배선 여유는 모델링하지 않은
시뮬레이션 설계안이다. 이동 베이스 운용 및 모든 물체의 수거 성공은 별도 검증이 필요하다.
각 함의 외형은 180×170×120 mm,
벽 두께 8 mm이며 바닥과 네 벽이 있는 열린 충돌 형상이다.
기존 홈 자세를 유지하는 MuJoCo GUI만 실행하며 인식·자동 수거는 시작하지 않는다.
이 설정은 배치 검토용 시뮬레이션 시안이며 실제 장착 강도/하중과 수거 경로는
검증되지 않았다. 자동 수거와 일반 파이프라인 launch도 동일한 후면 설정을
기본으로 사용한다. 배치 통일은 수거 경로 검증 완료를 뜻하지 않는다.
선택적 `rim_rgba`는 벽 두께 안쪽의 둥근 시각 테두리를 추가하며 질량/충돌은
추가하지 않는다. 물리 형상은 기존 바닥과 네 벽으로 유지한다.

`study_cafe_backend.launch.py scheduled_cameras:=true`는 sorting observer hardware의
카메라별 촬영 주기 제어를 켠다. sorting_bins_config가 있어야 해당 hardware를 사용한다.
직접 backend를 실행할 때 기본값은 false다.
`config/sorting_wrist_cameras.yaml`은 양팔 RGB optical frame의 nominal CAD 장착값이다.
수거 launch의 TF 입력이며 실제 로봇에서 측정한 hand-eye calibration이 아니다.

`scenes/study_cafe_grasp_execution.xml.in`에는 컵–양손 그리퍼의 10개 접촉쌍을
명시해 미끄럼 마찰을 기존 접촉값 3에서 6으로 높인 시험 설정을 둔다.
`friction="6 6 0.2 0.1 0.1"`이며 비틀림/구름 마찰과 기본 접촉 softness는
유지한다. 컵–책상, 다른 물체, 질량·모터 힘·형상은 바꾸지 않는다.
이는 실측 재질 계수가 아닌 시뮬레이션 민감도 시험이며 실제 패드 성능을 보장하지 않는다.

분리 수거용 `study_cafe_backend.launch.py`에서 수거함 config를 전달한 경우,
`sorting_contact_diagnostics:=true`로 출력 전용 접촉 진단을 켤 수 있다.
기본은 false이며 `/simulation/contact_diagnostics` 계약과 한계는
`cleany_mujoco_observer/README.md`를 따른다. 로봇/물체 물리 상태나 perception
입력을 바꾸지 않는다.

## 분리 수거 fixture

`study_cafe_backend.launch.py sorting_bins_config:=<path>`는 materialized
scene의 chassis 기준으로 바닥과 네 벽을 갖는 좌우 수거함을 추가한다.
`config/robot_top_bins.yaml`의 +Y가 분실물함, -Y가 쓰레기함이다.
빈 인자(기본값)는 기존 장면을 그대로 사용한다. template/materializer 외의
canonical 로봇 mesh/scene asset은 변경하지 않는다.

수거함은 실제 MuJoCo 충돌 형상이다. 물체를 순간 이동하거나 그리퍼에
weld하는 기능은 추가하지 않았다. 설정 시
`cleany_mujoco_observer/ObservedMujocoSystem` hardware wrapper를 선택한다.
설치된 backend의 제어/물리 동작을 유지하며, 잠금된 `get_data` 복사로
평가용 관측을 수행한다. 동시 갱신되는 vendor plugin 배열을 직접 읽지 않는다.
`placement_verifier`는 `/simulation/sorting_ground_truth`를 관찰하고
`/sorting/verify_placement` 요청에 release 이후의 올바른 수거함 내부
안착 여부를 응답한다. 이는 시뮬레이션 평가용 정답 정보이며 RGB-D 인식이나
grasp/경로의 입력으로 쓰지 않는다. 동작 연결은 skill executor의
`make sim-mujoco-sorting`을 사용한다(전체 집기·놓기 검증은 진행 중).

현재 vendor renderer는 headless여도 GLFW 연결을 필요로 한다. 이 VM에서는
유효한 `DISPLAY=:0`을 전달해야 RGB-D가 발행된다. GUI 창을 켜야 한다는
뜻은 아니며 display가 없으면 모델 준비 완료 뒤에도 RGB-D timeout이 난다.

XLeRobot MuJoCo 시뮬레이션을 ROS 2 `ament_python` 패키지로 연결한다.

## 상태와 책임

유지하는 장면은 스터디카페 두 가지다.

- `scenes/study_cafe.xml.in`: 방·책상·물체 배치를 확인하는 관찰 장면.
- `scenes/study_cafe_grasp_execution.xml.in`: 고정 베이스에서 집기·분류·놓기를 실행하는 장면.

`mujoco_study_cafe.launch.py`는 custom bridge를 통해 관찰 장면을 실행한다.
`study_cafe_backend.launch.py`는 `mujoco_ros2_control`과 팔·그리퍼 controller를
실행한다. 공통 기본 장면도 스터디카페이며, 두 backend를 동시에 실행하지 않는다.
캔·박스·RGB-D pick·hand-eye 전용 시뮬레이션과 비활성 OBJ는 제거했다.
현재 물체는 종이컵·레고·휴지·마우스이며 외부 OBJ/texture는 마우스만 사용한다.
로봇 CAD mesh는 공통 모델에 필요한 `cleany_description`에서 관리한다.

## 실행과 테스트

### 되돌릴 수 있는 고정 책상 최적화

`sim_performance_profile`은 임시 materialized scene에만 적용한다. 기본값
`baseline`은 기존 설정을 그대로 유지한다. 소스 MJCF, mesh, layout은 수정하지 않는다.

| 프로필 | 배경 충돌 | 그림자 맵 |
| --- | --- | --- |
| `baseline` | 기존 그대로 | 4096 |
| `tabletop_collision` | 먼 정적 가구만 비활성화 | 4096 |
| `tabletop_fast` | 먼 정적 가구만 비활성화 | 2048 |

`config/tabletop_performance.yaml`의 반경 2m 바깥에 **형상 전체가** 위치한
책상·모니터·파티션·벽만 대상으로 한다. 보수적인 경계구를 사용하므로 근처
가구와 큰 벽은 유지될 수 있다. 바닥, 로봇, 수거함, 물체 및 명시 contact pair는
유지한다. 질량·마찰·관절 제한·timestep·solver·카메라 해상도/FOV·추적 FPS·
MoveIt/OctoMap은 변경하지 않는다. `tabletop_fast`는 GUI와 센서 영상 모두에서
그림자 정밀도가 낮아질 수 있으므로 인식 회귀 검사가 필요하다.

이 프로필은 chassis-world weld가 있는 `study_cafe_grasp_execution` 장면에만
허용한다. 이동 베이스·내비게이션·실제 하드웨어에는 적용하지 않는다.
시뮬레이션 시간 배속이나 로봇 이동 속도 상향 기능이 아니다.

2026-09-11 시험에서 먼 배경 충돌 602개를 비활성화해도 물리 step 시간은
유의미하게 줄지 않았다. 그림자 축소의 헤드 RGB 렌더 시간 감소는 약 12%였으나
전체 수거 시간 단축은 확인하지 못했다. `tabletop_fast` headless 시험은 레고
수거 후 컵 접근에서 관절 목표 오차로 실패했고, 비교 `baseline` 실행은 그 컵을
수거했다. 원인을 최적화로 확정한 것은 아니지만 **기본 프로필은 baseline으로
유지하며 fast는 실험용**이다. 같은 파지 성공률이 검증됐다고 해석하지 않는다.

```bash
# 적용 (기존 GEMINI_API_KEY / DISPLAY 등 실행 환경 유지)
make sim-mujoco-sorting SORTING_ARGS='sim_performance_profile:=tabletop_fast headless:=true use_rviz:=false use_image_view:=false'
# 원복: 실행을 종료한 뒤 baseline으로 재시작 (파일 복원 불필요)
make sim-mujoco-sorting SORTING_ARGS='sim_performance_profile:=baseline headless:=true use_rviz:=false use_image_view:=false'
# 그림자 품질을 유지하며 배경 충돌만 줄이기
make sim-mujoco-sorting SORTING_ARGS='sim_performance_profile:=tabletop_collision'
# 별도 정지 장면 성능 비교: 다른 시뮬레이터를 종료한 뒤 실행
make profile-mujoco-tabletop BENCHMARK_ARGS='--output /tmp/tabletop_benchmark.json'
```

벤치마크는 현재 launch 초기자세에서 세 프로필을 교차 순서로 반복한다.
물리 step과 640×480 카메라별 RGB 렌더+readback을 따로 측정하며,
MuJoCo Python 버전도 기록한다. ROS backend/vendor 버전, depth readback,
인식 모델, GUI, 실제 파지 접촉 부하를 포함한 전체 수거 시간과는 다르다.
일괄 속도 향상률로 해석하지 않는다.

아래 명령은 레포지토리 루트에서 실행한다.

```bash
make sim
make sim-mujoco-study-cafe
make test-mujoco
```

`sim-mujoco-study-cafe`는 Gazebo 개발공간의 12.26×10.94 m 방과 동일한 좌표에
벽 4개, 책상 48개, 파티션 24개, 모니터 48개를 띄우며 의자는 생성하지 않는다.
기존 Gazebo spawn에서 가장 가까웠던 43번 의자 자리를 기준으로 끌리니 1대를
책상을 향하도록 배치한다. 의자 중심에서 책상 반대 방향으로 0.22 m 이동하며
후퇴 거리는 `robot_center_rearward_offset_m`으로 설정한다. 로봇 앞 43번 책상의 현재
물체는 손잡이 없는 종이컵, 빨간 2×4 레고 블록, 흰 휴지뭉치, 무선 마우스다.
마우스는 [Computer Mouse — CreativeTrio](https://poly.pizza/m/V2Ebx3pvo4)의 CC0
메시와 원본 텍스처를 사용하고 110×65×35mm로 정규화했다.
원본/변환 SHA256과 변경 사항은 `config/study_cafe_assets.yaml`, 재현 도구는
`tools/convert_computer_mouse.py`에 있다.
마우스는 책상 중심 기준 `[-0.14, -0.095]m`에 배치한다. 이전 위치에서
로봇 방향으로 8cm 당긴 시험 배치이며 크기·질량·마찰은 유지한다.

| 물체 / body ID | 외형 크기 | 물리 모델 |
| --- | --- | --- |
| 종이컵 / `study_cafe_cup` | 기존 80% 크기에서 추가 10% 축소: 윗지름 61.2, 밑지름 39.6, 높이 68.4, 벽 0.504 mm | 열린 원뿔대 벽 32개와 바닥, 8 g 유지 |
| 레고 / `study_cafe_lego` | 2배 확대 후 높이만 추가 10% 확대: 몸체 63.6×31.6×21.12 mm, 돌기 포함 높이 25.08 mm | 본체 box와 8개 돌기 cylinder, 2.3 g 유지 |
| 휴지뭉치 / `study_cafe_tissue` | 60×50×45 mm | 불규칙 삼각형 표면과 convex-hull collision, 1 g |
| 무선 마우스 / `study_cafe_mouse` | 110×65×35 mm | CreativeTrio CC0 mesh·원본 텍스처와 convex-hull 충돌, 120 g (시뮬레이션 가정) |

`cleany_mujoco_sim/tabletop_shapes.py`가 설정값에서 결정적으로 MJCF mesh/primitive를
생성한다. 컵은 수거함 입구의 보수적 크기 판정 여유를 확보하기 위해 기존 형상의
80%로 축소한 뒤 접근 비교를 위해 추가 10% 축소했다(최초 대비 72%). 질량·마찰·위치·파지 설정은 유지하며, 축소만으로 실제 파지/수거
성공을 보장하지 않는다. 네 물체 모두 free joint가 있으며 강제 고정/attach하지 않는다. 종이컵은
안쪽을 채운 cylinder가 아니므로 입구로 들어가는 접촉도 가능하다. 다만 종이컵과 휴지는
눌리거나 구겨지지 않는 **강체 근사**이며, 레고 밑면 결합 튜브와 로고는 생략했다.
질량과 종이컵/휴지 크기는 시뮬레이션 가정이며 실물 측정값이 아니다. 레고는
[실측 치수](https://www.cailliau.org/Alphabetical/L/Lego/Dimensions/More%20Dimensions/BBEditPreviewTemp.html)의
8 mm 돌기 간격, 4.8 mm 지름, 1.8 mm 돌기 높이를 기준으로 현재 모두 2배
확대해 각각 16 mm, 9.6 mm, 3.6 mm를 사용한다. 시각·충돌 형상을 함께 확대하며,
크기에 따른 파지 비교를 위해 질량·위치·마찰·파지 설정은 유지한다.
이는 실제 규격 크기가 아닌 진단용 확대 모델이다.
파지 실행 launch의 우측 팔 mirrored home 자세는 wrist-roll을 `-1.58 rad`로
시작하는 기존 설정을 유지한다.
위치, 질량, 색상과 collision 크기는
`config/study_cafe_layout.yaml`에서 조정할 수 있다. 기본 위치는 640×480
`head_realsense_rgb` 카메라와 `head_tilt_joint=1.00 rad` 조건에서 네 물체의 전체
물체가 화면 경계 안에 들어오도록 맞춰져 있다.

실제 집기 실행은 `scenes/study_cafe_grasp_execution.xml.in`을 사용한다. 일반 관찰 장면과
같은 배치를 유지하면서 chassis를 world에 고정한다. 종이컵/레고와 양쪽 fixed/moving jaw
사이의 고마찰 contact pair는 복합 collision의 모든 부분으로 확장한다(컵 330개,
레고 90개). 컵의 sliding friction은 6이다. 레고는 열린 fixed jaw에 남는 현상을
줄이기 위해 명시 pair friction을 `(1.5, 1.5, .001, .0001, .0001)`로 설정한다
(이전 `(5, 5, .4, .15, .15)`). 좌우 sliding 마찰 외에 비틀림/구름 저항도 낮췄다.
두 팔의 본체/돌기 pair에 동일하게 적용하며 파지와 방출 중 값을 전환하지 않는다.
이는 시뮬레이션 튜닝값이지 실제 종이/플라스틱의 측정 마찰계수가 아니다.
물체 weld, 중력 보상, 강제 낙하 힘 또는 방출 시 collision 비활성화는 사용하지 않는다.
2026-09-11 headless 레고 단독 파이프라인에서 파지·운반·배치 성공과
열림 시작 후 약 0.10 simulation 초에 하강, 완전 열림/팔 복귀 전에 낙하를 확인했다.
진단은 `artifacts/lego_release_fix_20260911/REPORT.md`에 기록했으며,
다른 자세/물체의 성공률 보장은 아니다.
얇은 강체 컵 벽의 수치적 관통을
줄이기 위해 컵 pair는 `solref="0.002 1"`, `solimp="0.99 0.9999 0.0001"`을 사용한다.
이는 종이의 찌그러짐을 재현하는 설정이 아니며 실제 컵 파지력 검증을 대체하지 않는다.

컵의 `geometry.collision_bottom_clearance_m` 기본 배치값은 `0.00025`m다.
충돌 벽 32개의 외측 아래 꼭짓점을 원뿔대 경사를 따라 0.25mm 올려, 똑바로 선 컵의
하중을 바닥 원판이 받도록 한다. RGB 시각 메시, 열린 내부, 위쪽 파지 면, 질량 8g,
마찰 및 solver 설정은 유지한다. 벽 수를 줄이는 것보다 중복된 바닥 접촉 제거가
효과적이어서 충돌 벽은 32개를 유지한다. 값은 0 이상 바닥 두께(0.504mm) 이하만
허용하며, 생략하거나 `0`을 설정하면 기존 충돌 형상으로 돌아간다.

2026-09-29 MuJoCo 3.7 정지 장면 비교(실제 launch home, 1 simulation 초 settle 후
2초 측정, 순서를 교대해 3회 반복, ROS·렌더링 제외)에서 컵 접촉은 69→5개,
전체 접촉은 88→24개, 물리 step 중앙값은 1.306→0.230ms였다(약 82% 감소).
수정 후 측정 구간 위치 변화는 약 0.0012mm였다. 이는 **정지 상태 물리 계산**의
개선이며, 전체 인식·집기 파이프라인이나 GUI가 같은 비율로 빨라진다는 의미는 아니다.
비교 원자료와 재현 스크립트는 로컬 `artifacts/cup_contact_optimization_20260929/`에
보관한다. `test/test_tabletop_shapes.py`는 시각 형상 불변, 열린 입구, 파지 높이의
측면 연속성, 바닥 지지와 접촉 감소를 검사한다.

휴지와 마우스 강체 proxy도 각각 양쪽 jaw와의 10개 pair에 같은 강성을 적용한다. 기본 혼합
마찰 `(3, 3, .2, .1, .1)`과 질량/형상/모터 한도는 유지한다. 부드러운 접촉에서
관측된 휴지 약 18mm, 지갑 약 2.1mm 메시 관통을 줄이기 위한 설정이며
실제 휴지나 마우스의 변형 모델은 아니다.
마우스는 시각 메시를 collision mesh에도 공유한다(MuJoCo convex hull).
오목한 버튼 장식까지 충돌 분해한 모델은 아니다.
머리 카메라는 materialization 시
`640×480`, `fovy=42°` 계약을 검사해 RGB-D driver의 1×1 기본 해상도 사용을 막는다.

조명은 `config/study_cafe_lighting.yaml`에서 설정하며 일반 관찰/집기 장면과 GUI/RGB-D
렌더에 공통 적용한다. 기존 canonical robot의 world light는 materialized copy에서만
제거하고, 작업 책상이 포함된 3열×2행의 인접 책상 6개에 하나의 spotlight를
배치한다. 중심은 `(-4.33, -3.17, 3.30)m`, 수직 아래 방향, cutoff 40°,
exponent 0이다. 책상 높이에서 반경 약 2.16m 범위를 같은 각도 강도로 비춘다.
4096 shadow map을 방 전체 대신 이 구역에 집중해 그림자 계단 현상을 줄인다.
방 전체 directional 보조광과 ambient는 유지하며, 그림자 광원은 하나다.
카메라 headlight는 ambient 0.12, diffuse 0.08, shadow map은 4096이며
전역 directional shadow half extent는 12 m다. 보조광의 그림자는 꺼져 있다. 이는 실측 조도나
실내 간접광을 계산하는 모델이 아닌 근사다.
기존 headlight와 기본 광원의 합산으로 흰 상판이 포화되고
그림자 대비가 사라지는 문제를 줄이기 위한 설정이지 실측 조도/카메라 보정값은 아니다.
[MuJoCo headlight는 자체적으로 그림자를 만들지 않는다](https://mujoco.readthedocs.io/en/stable/XMLreference.html#visual-headlight).
그림자 광원은 추가 렌더 패스를 사용하므로 렌더링 비용이 증가할 수 있다. 물체 크기,
마찰, 카메라 intrinsics, 검출 confidence 및 파지 안전 기준은 조명 변경으로 바꾸지 않는다.

Gazebo plugin·sensor는 옮기지 않으며, Fuel에서 내려받는 의자 mesh 대신 Gazebo
collision 치수와 같은 MuJoCo primitive를 visual/collision로 사용한다. 원본 배치는
`config/study_cafe_layout.yaml`에 독립 실행용으로 복제하며, 테스트에서 Gazebo 원본
YAML과의 일치 여부를 확인한다.

Tabletop asset 출처와 라이선스는 `config/study_cafe_assets.yaml`에 기록한다. 현재
외부 mesh는 Computer Mouse(CreativeTrio, CC0)이며 원본 UV texture를 사용한다.
과거 Coffee Mug(Michael Fuchs, CC BY 3.0)와
Smartphone(smallbigsquare, CC0)의 OBJ, URL, 변환 내역, SHA-256은 이력용으로
보존하지만 현재 장면에 로드하지 않는다.

새 형상의 장면 컴파일, 실물 크기, 열린 컵 충돌체, 물리 settle, 카메라 시야는
`test_study_cafe_scene.py`/`test_tabletop_shapes.py`와 `make test-grasp-pregrasp`로
검사한다. 이 검증은 YOLOE 검출이나 집기 성공 검증을 대신하지 않는다. 기존 머그컵의
수거 성공/시간 측정 결과를 새 종이컵·레고·휴지에 적용하지 않는다. 실제 크기 레고는
sorting의 기존 50 mm 최소 파지 후보 폭보다 작으므로 자동 집기 대상에서 제외될 수 있다.

직접 launch할 수도 있다.

```bash
ros2 launch cleany_mujoco_sim mujoco_study_cafe.launch.py headless:=false
```

전체 수거와 인식 파이프라인은 레포지토리 루트에서 실행한다.

```bash
make sim-mujoco-sorting
make sim-mujoco-pipeline
make test-grasp-pregrasp
make test-mujoco
```

`make test-mujoco`는 현재 장면과 custom bridge의 전체 단위 테스트를 검사한다.
이전 데모와 빈 방의 MuJoCo 주행 runtime 검사는 제거했다. 공통 명령 검증·timeout·
휠 매핑 단위 테스트는 유지하며 전체 물리 수거 성공은 별도 검증 대상이다.

## ROS contract

스터디카페 수거 backend는 `mujoco_ros2_control/ros2_control_node`,
`robot_state_publisher`, joint-state broadcaster와 양팔·그리퍼 trajectory controller를
실행한다. `config/study_cafe_ros2_controllers.yaml`에서 controller 설정을 관리한다.
초기 관절 값은 임시 장면의 `handeye_ros2_control_home` keyframe에 기록한다.
이 이름은 vendor plugin 호환을 위해 유지하는 내부 keyframe 식별자다.

스터디카페 통합 launch는 head RGB-D와 nominal 손목 TF를 연결한다.
선택형 `camera_contract_adapter`는 `config/wrist_camera.yaml`에서 왼손목 camera
intrinsics와 topic 계약만 읽는다. 보정판·이전 hand-eye 장면에는 의존하지 않는다.
RGB-D의 원본 촬영 시각을 보존하며 실제 카메라 보정값을 의미하지 않는다.

## 베이스 구동 모델

각 휠은 독립적인 `PG42-4266-1270NE` output-shaft DC motor 모델을 사용한다.
actuator 제어 입력은 `rear_left_drive`, `rear_right_drive`, `front_left_drive`,
`front_right_drive`의 모터 단자 전압이다. `base_link +X`가 전방이며, 양의 yaw는
`+Z` 기준 반시계 방향이다.

- 정격 공급 전압: `12 V`
- 기어박스: `61:1`; 제조사 출력 토크에는 `72%` 효율이 반영되어 있다.
- 제조사 정격 출력: `103 rpm`에서 `2.94 N.m`
- 계산한 무부하 출력: `7000 / 61 rpm` (`12.017 rad/s`)
- 10% 마진을 적용한 시뮬레이션 운용 한계: `10.8 V`, `2.646 N.m`
- 디레이팅한 정격점: `92.7 rpm` (`9.708 rad/s`)
- 디레이팅한 무부하 평형점: `103.28 rpm` (`10.815 rad/s`)

MJCF actuator는 기어박스 출력축에서 직접 모델링하므로(`gear=1`) 감속비와 효율을
다시 적용하지 않는다. 매 physics step마다 실제 joint 속도를 읽고, feed-forward와
anti-windup을 포함한 휠별 PID controller로 `-10.8~10.8 V` 전압을 계산한다.
PID gain은 MuJoCo DC motor step response를 기준으로 한 시뮬레이션 값이며 실제
ESP32 controller에 그대로 사용하지 않는다.

## 팔 서보 모델

양팔은 Feetech 12 V 시리얼 서보를 사용한다.

- left/right shoulder pitch와 elbow pitch: `STS3250`
- shoulder rotation, wrist pitch/roll, jaw, head pan/tilt: `STS3215`

모델의 출력 한계에는 제조사 사양 대비 10% 운용 마진을 적용한다. 위치 actuator와
joint force 한계에는 최대 정지 토크의 90%를 적용한다. 전류, 발열 및 2초 과부하
차단 동작은 아직 모델링하지 않았다.

## 실행 인자

`mujoco_study_cafe.launch.py`는 `headless`를 제공하며 기본은 false다.
공통 custom bridge인 `mujoco_sim.launch.py`의 `scene_path` 기본값도
`scenes/study_cafe.xml.in`이다. 주행 관련 bridge parameter는 이 관찰 backend의
공통 기능으로 유지한다.

`study_cafe_backend.launch.py`의 주요 인자:

- `scene_path`: 기본 `scenes/study_cafe_grasp_execution.xml.in`.
- `controller_config`: 기본 `config/study_cafe_ros2_controllers.yaml`.
- `camera_config`: 기본 `config/wrist_camera.yaml`.
- `headless`, `sim_speed_factor`, `sim_xmodifiers`: viewer·속도·XIM 설정.
- `sorting_bins_config`, `scheduled_cameras`, `sorting_contact_diagnostics`: 수거함·카메라 스케줄·관측 설정.
- `camera_name`, `camera_frame_name`, `color_image_topic`, `camera_info_topic`, `depth_image_topic`: renderer 연결.
- `enable_camera_contract_adapter`: 기본 false. 통합 launch는 head camera topic을 직접 연결한다.
- `enable_gripper_controllers`, 양팔 `*_initial`, `head_tilt_initial`: 제어 interface와 초기 자세.

수거 전체의 초기 자세와 센서 연결은 `cleany_skill_executor`의
`study_cafe_sorting.launch.py` 및 `study_cafe_nearest_grasp_demo.launch.py`를 따른다.

## 시뮬레이션 확장 API

같은 process의 sensor adapter는 `MujocoSimNode.simulation_context`에서
`MujocoSimulationContext`를 받아 MuJoCo model과 최신 data를 조회할 수 있다. 두
native handle은 렌더링 등 MuJoCo API 호출에 직접 사용할 수 있지만 adapter가 물리
상태를 변경하지 않는 read-only 계약을 따른다.

`MujocoSimNode.add_step_observer(observer)`에 `StepObserver` 구현을 등록하면 각 ROS
timer tick의 모든 물리 substep이 끝난 직후 한 번 `after_step(context, stamp)`가
호출된다. callback은 기존 ROS 상태 토픽 발행 전에 동기적으로 실행되며 예외를
숨기지 않는다. observer는 callback을 짧게 유지해야 한다.

## 범위

구현된 기능:

- `cleany_description`의 authoritative MJCF를 simulator-owned scene에 load
- 독립적인 PG42 motor를 사용하는 네 개의 5-inch mecanum drive wheel 시뮬레이션
- ROS timer로 simulator step, joint state·odometry·laser scan·TF 발행
- 시뮬레이션 시험용 직접 joint position 명령
- `/cmd_vel` 검증·제한·timeout과 메카넘 역기구학
- 휠별 PID, feed-forward, anti-windup, 전압 제한을 통한 actuator 제어
- 좌·우 5축별 표준 `FollowJointTrajectory` action과 12개 arm/gripper joint state를
  제공하는 배타적인 MuJoCo `ros2_control` backend
- Humble release `MujocoCameras` 기반 left wrist RGB/CameraInfo와 public contract
  normalizer
- 스터디카페 head·손목 RGB-D renderer 연결과 카메라 스케줄링
- 관측 wrapper를 통한 평가 전용 수거함·접촉·안착 정보

아직 구현되지 않은 기능:

- `cleany_robot_interface` 또는 Mission FSM의 운영 인터페이스 연동
- 실제 encoder noise, 전류, 열, 과부하 차단 동작
- 베이스 또는 매니퓰레이터용 하드웨어 현실성 기반 controller

## 공통 RGB-D 파이프라인 연결

`study_cafe_backend.launch.py`에 `color_image_topic`, `camera_info_topic`,
`depth_image_topic`을 명시하면 선택 MJCF camera 출력을 해당 ROS 토픽으로
직접 remap한다. 비워 두면 `wrist_camera.yaml`의 internal 토픽을 사용한다.
custom 토픽 사용 시 `enable_camera_contract_adapter:=false`가 필요하다.
이 옵션은 영상 전달 경계만 바꾸며 simulator geometry/물체 정답을 알고리즘에
주입하지 않는다. head 고정 camera 보정 TF의 실제 로봇 정확성을 뜻하지 않는다.

레포 루트의 `make sim-mujoco-pipeline`은 학습 perception과 sensor-only
MoveIt 지도를 포함하는 plan-only GUI 실행이다. 장면만 띄우는
`make sim-mujoco-study-cafe`는 장면 관찰용이다.

## 관련 KB와 문서 갱신

- [Robot Platform XLeRobot](../../../docs/cleany-docs/20_TECHNICAL/04%20-%20Robot%20Platform%20XLeRobot.md)
- [Navigation and Mapping](../../../docs/cleany-docs/20_TECHNICAL/05%20-%20Navigation%20and%20Mapping.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)

발행 topic, launch parameter, 시뮬레이션 모델 가정 또는 테스트 명령이 바뀌면 이
README도 갱신한다. 시뮬레이션 하드웨어 파라미터를 관련 KB 결정 없이 확정된 실제
하드웨어 사양으로 표현하지 않는다.
