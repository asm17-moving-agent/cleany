# Desktop M1 진단·개선 작업 기록

초기 상태와 모든 실패 과정을 아래에 보존한다. 최종 결과는 REPORT.md와 RESULTS.json/CSV를 기준으로 한다. 원래 dirty checkout과 기존 실험은 수정하지 않았고 물리 로봇/노트북 실행 프로세스에는 명령을 보내지 않았다. 노트북에서는 사용자가 Tailscale 인증한 뒤 최적화 소스와 설정 파일만 읽어 복사했다. git push 없음.

## 환경과 기준

- 호스트 x86_64, `cleany-ros-humble-cdi`, Fortress 6.18.0, Nav2 1.1.20.
- 컨테이너에서 NVIDIA RTX 5070 / 610.57.04 확인. 실제 Ogre2 GL_RENDERER도 RTX 5070 (`provenance/ogre2.log`). 호스트 기본 NVML은 615/610 버전 불일치; 기존 컨테이너의 격리 드라이버를 사용했고 시스템 드라이버 변경 없음.
- Cleany 런타임: `controller-comparison-20261013/cleany-data/builds/source/install`, source HEAD b3ca816. 수정된 메인 `cleany/`는 사용·변경하지 않음.
- MPPI는 노트북 최적화 소스를 복사해 이 호스트에서 빌드. 원본 `provenance/mppi-original/`, 계측 기준선 `src/`, 후보 `candidate*/src/`. 런타임별 실제 로드 SO 해시를 result.json에 기록.
- 전체 18F canonical world의 모델 집합 및 모델 XML을 비교하여 해당 목표 의자만 제거됐음을 검증. URI만 현재 호스트 경로로 바꿈. 접촉 관찰 센서 추가, 물리 geometry/physics 유지.
- 지도는 노트북과 같은 2.5cm map의 2x2 occupied union → 5cm 변환. 이전 실험의 별도 5cm raster와 혼용하지 않음.
- 정확한 footprint / padding 0, 휠 5rad/s coupled projection, goal 0.30m/10°, stock 5cm/0.5m omni lattice, reverse=true, smooth=false, 80s, max_iterations=-1 유지.
- 기존 외부 pan/2D/3D guard는 shadow 출력. Nav2 내부 충돌 검사 유지. AMCL 0.01m/0.01rad + 1sim초 nomotion update, GT는 관찰만 사용.

## 완료한 증거

`native/native-maneuver-check-result.json`: 이 호스트 라이브러리로 재컴파일. 현재253, 경로0/29, 연결보간0/244, 제자리회전8/31, 방향고정이동35/244. ARM 바이너리는 실행하지 않음.

`native/replay/diagnosis.json`: 노트북 정체 costmap/path를 고정해 native optimizer/critics 재생. 고정 Gaussian batch, 측정 속도 0, 위치 고정이라는 한계가 있는 알고리즘 시험이다. 첫 후보에서 필요한 +19.33° yaw를 Gaussian이 생성하지 못함(−6.69~+5.54°). 기존 비용 항목 그대로 coherent path 후보를 넣으면 총비용16.704→9.135, 충돌 벌점 없음. 최저 비용 후보의 PathFollow14.774→5.808, Cost1.466→2.015, Twirling0.464→0.844. PathAlign은 원래 첫 배치에서 furthest gate 때문에0. 단일 critic이 가능한 결합 동작 자체를 배제한다는 증거는 아님.

## 실제 주행 (계속 갱신 예정)

- `baseline_m1`, `baseline_m1_v2`: startup 검증만 진행. 부동소수점 footprint 표기 차이, 미사용 파라미터 검증 문제를 수정한 후 별도 경로로 재시작. 성능 표본에서 제외.
- `baseline_m1_v3`: 실제 HOME→M1 시도. Nav2 status6, 31.78sim초, RTF0.3563, GT오차13.599m/81.822°. 복도에서 모든 MPPI 후보가 충돌 판정된 후 재계획 시작점 lethal로 종료. 문 앞까지 도달하지 않았음. contact floor 양성대조 4개 휠에서 기록, 비바닥 접촉 기록0. 전신 무접촉을 독립적으로 증명했다는 뜻은 아님.
- `pathseed_m1_v1`: startup assertion 수정이 필요해 목표 전 종료. 제외.
- `pathseed_m1_v2`: coherent path proposals64 + 경로 진행량12초 + 주기적재계획/후퇴. 복도 곡선에서 정렬 부족으로 실패. 후퇴는 native collision 검사에 의해 거부됨. `failure_snapshot.png` 참고.
- `pathseed_m1_v3`: proposals128, PathAlign weight12, weighted mean/filter 출력의 native dense collision 검증 추가. 복도 통과 후 문 앞에서 Nav2 6으로 실패. GT 1.088m/63.10°, 103.576sim초. 후퇴 충돌 검사 거부.
- `prealign_m1_v1`: M1 문 밖 [-17.125,0,pi] → 문 안 [-17.125,2,pi] → 기존 최종 목표를 같은 Smac/MPPI로 실행. 모든 단계 status4, 최종 GT 0.274276m/7.62939°, 126.769sim초/299.954wall초, RTF0.42263. 실제/명령 휠 최대4.90846rad/s, 관찰 비바닥 접촉0. 최종 접근 중 planner 실패로 BackUp0.35m가 실행·성공한 후 재계획·도착. 문 통과 중 yaw를 강제로 고정하지 않으며 Smac의 비스듬한 중간 자세도 사용됨. 사전 정렬 + 출구 목표 조합의 실제 성공이며 각 변경의 독립적 인과 효과를 주장하지 않음.
- `regression8`: 개선 설정(prealign.yaml + portal_approaches.yaml + candidate_v2)을 고정하고 0..7 각1회 순차 실행 시작. 추가5회 반복 없음.

## 구현 시험

`native/test_path_seed`: 이동+회전 결합, wheel envelope, empty path, yaw wrap 통과.
`native/test_path_progress`: arc projection, 회전만 하는 정체, 재계획으로 타이머 무한 연장 방지, 실제 진행, 최종 yaw 정렬 통과.

## 초기 남은 완료 조건 (이후 수행 완료)

실제 M1 문 통과·도착, 공통8목표 각1회, 실패 원인 및 성능 JSON/CSV, 재현 README/적용 설정/패치, 실제 주행 데이터를 이용한 GIF 또는 영상, 최종 한국어 REPORT. 추가5회 반복은 시행하지 않는다. 실패를 성공으로 집계하지 않는다.

## 공통8 1차 결과 및 후속 수정

`regression8` 완료: 0,2,3,4,5,6 성공(6/8). 1은 controller 도착 후 concurrent planner abort가 복구/실패로 전파. 7은 prealign 성공 뒤 문 통과 중 다시153°가 되어막힘. 앞선 M1 1회 성공만으로 안정성 확보를 주장하지 않음. `failure_analysis.json`에 원인/증거 보존.

`recovery_keep_path.xml`은 새 계획 성공시에만 typed blackboard 경로를 교체. Nav2 on_aborted/halt가 output을 비우므로 별도 temporary path가 필요하다. native 실제BT controls로 회귀시험 통과. `portal_approaches_v2.yaml`은 lattice cell center를따라0.3m 짧은중간목표, checker2cm/2°, `prealign_v2.yaml` XYlock.01/.015m. 각1845개의 전역·지역 native검사 모두충돌0. 최종30cm/10°유지. 후속시험 fault0→turn1→M1만시작, 8목표추가반복없음.

`fault_recovery` 성공:12..28sim초 명령정지,계획5개갱신중에도첫failedcompute23.499sim초(주행시작11.312), route12초timeout→0.35402m후퇴→status4/GT.24902m/.78317°. `replan_fix_turn_arrival` status4/GT.03807m/6.89224°로회귀수정확인.

`replan_fix_m1` 짧은목표v2는prealign/portal1/portal2성공뒤portal3에서정체실패. yaw≈180°유지했으나주기적Smac재계획이현재.825→뒤로.525→회전/우회→목표.925의loop로바뀜. 마지막1000개CostCritic은모두0, 끝점은전부현재보다뒤쪽; 단순충돌벌점문제가아님. 라이브lastsnapshot/후보CSV보존. stockprimitive에서yawpi lateral끝점±.15m이고격자위상단거리재계획이loop를만들수있음. 새`portal_approaches_v3.yaml`은문안30cm구간만초기Smac경로를유지하며,충돌/12초진행실패시에bounded복구후재계획. 접근과최종goal에는계속주기적재계획. MPPI소스/충돌검사/footprint변경없음. `portal_hold_m1` 별도실행중.

## 최종 검증 완료

`portal_hold_m1` 모든9단계status4. GT0.0508387m/8.397209°,138.956sim초/326.594835wall초,RTF0.425469. 명령휠4.85453rad/s,측정joint4.870126rad/s,비바닥접촉기록0,deadline경고0. compute3.431/20.034/21.848ms(median/p95/p99). 정지8.65초/출발13회/선형jerkRMS1.673으로중간목표정지비용을숨기지않음. 최초8개6/8,실패한1/7은후속개별재시험성공. v2중간재실패도그대로보존. 추가5회묶음없음.

한국어REPORT,전체16개측정RESULTS JSON/CSV,최초8개summary JSON/CSV,최종설정모음,실제GazeboGT GIF,원인그림,재현README,sourcepatch와dry-run검증,원시costmap/critic/rosbag/log저장. mainHEAD는완료점검시별도checkpoint커밋으로변경되어있었으며이작업은편집/커밋하지않음. 실험에사용한b3ca816소스는초기와동일. 모든소유시뮬레이션은종료됨.
