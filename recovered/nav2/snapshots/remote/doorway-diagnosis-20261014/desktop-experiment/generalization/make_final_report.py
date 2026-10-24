from pathlib import Path
import json,datetime
r=Path(__file__).resolve().parent
runs=lambda folder:[json.loads(p.read_text())|{'run':p.parent.name} for p in sorted((r/folder).glob('*/metrics.json'))]
old=runs('regression8');new=runs('diverse8');complete=(r/'diverse8/completed.json').exists()
passed=sum(bool(m['success']) for m in new)
lines=['# 수동 경유점 없는 MPPI: 공통 8경로 검증','',f'작성: {datetime.datetime.now().isoformat(timespec="seconds")}', '',f'최종 후보 검증 상태: {"완료" if complete else "진행 중"}. 기록된 {len(new)}개 중 {passed}개가 Nav2 status=4 및 GT 위치≤30cm·방향≤10°를 동시에 만족했다.', '',
'## 변경과 결과의 범위','',
'모든 목표에서 M1 전용 사전 정렬, 중간 NavigateToPose 목표, 문 구간 전용 경로 유지 BT를 제거했다. 원래 HOME에서 기존 잠금 파일의 목표 하나만 보낸다. 최종 후보의 컨트롤러·비용 가중치·BT는 8개 모두 같다. 목표별로 바뀌는 것은 원래 지정된 목표와 제거할 목표 의자뿐이다.', '',
'첫 경유점 제거 시험은 **7/8**이었다. M1 단독 대조 실행은 성공했지만, 이어진 전체 묶음의 M1은 문 안쪽에서 실패했다. 단독 성공을 해결 근거로 삼지 않고 이 실패를 보존한 뒤 후보 생성과 복구를 수정해 전체 8개를 다시 실행했다.', '',
'### 최종 후보의 범용 변경','',
'- **경로 연결 후보 다양화:** 이전 128개 후보는 속도 8종의 반복이었다. 이제 전역 경로의 첫 지점뿐 아니라 가까운 여러 지점으로 연결하는 SE(2) 후보를 만든다. 선택 범위는 경로 arc 0.35m 이내·최대 16개 지점이며, 각 연결에서 8종 속도를 탐색한다. 전체 1000개 중 128개라는 비율은 그대로다. 나머지는 기존 Gaussian 후보다. 지도 좌표, 문 이름, 목표 번호를 참조하지 않는다. 모든 후보는 기존 critic과 휠 결합 제한을 거치며 출력은 native footprint 검사로 검증한다.',
'- **막힌 후퇴 뒤 재계획:** 기존 BT는 BackUp이 충돌 검사에 막히면 남은 재시도 횟수와 관계없이 전체 작업을 끝냈다. 이제 BackUp이 거부되면 0.5초 기다린 뒤 재계획한다. BackUp 충돌 검사는 그대로이고, 총 4회 복구 및 기존 12초 진행량 제한도 유지한다. 후퇴 실패를 주행 성공으로 기록하지 않는다.',
'- 기존의 성공한 재계획만 반영하는 처리, 평균/필터 출력 궤적 검증, 0.08m/12sim초 진행량 제한을 유지했다. PathAlign 가중치 12 등 기존 비용 설정은 바꾸지 않았다. 새 SE(2) 구간 투영 비용 함수와 높은 정렬 가중치는 비교 후 채택하지 않았다.', '',
'코드: `../candidate_diverse/src/nav2_mppi_controller`. 설정: [common.yaml](common.yaml), [recovery_replan.xml](recovery_replan.xml). 원본 대비 [소스 패치](candidate-diverse.patch), [변경 파일](changed-source-files.json), [patch dry-run](patch-dry-run.txt).', '',
'## 최종 공통 8경로 결과','',
'성공은 Nav2와 GT를 모두 만족해야 한다. 비바닥 접촉은 계측된 비자체 접촉 기록 수이며, 0건은 독립적인 비침투 증명이 아니다. 명령 제한과 Gazebo joint 응답은 따로 평가한다.', '',
'| 경로 | 결과 | GT 위치 cm | GT 방향 ° | sim s | wall s | joint 최대 rad/s | 비바닥 접촉 | compute p95 ms |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
for m in new:
 lines.append(f'| [{m["goal_name"]}](diverse8/{m["run"]}/drive.gif) | {"통과" if m["success"] else "실패"} | {m["position_error_m"]*100:.1f} | {m["yaw_error_deg"]:.2f} | {m["navigation_sim_s"]:.2f} | {m["navigation_wall_s"]:.2f} | {m["measured_joint_max_rad_s"]:.4f} | {m["contacts"]["non_floor_non_self_records"]} | {m["compute_timing"]["compute_us"]["p95"]/1000:.2f} |')
if new:
 lines+=['',f'관찰된 최대 명령 휠 속도: {max(m["max_command_wheel_rad_s"]["/cmd_vel"] for m in new):.9f}rad/s. 실제 joint 최대: {max(m["measured_joint_max_rad_s"] for m in new):.9f}rad/s.']
 lines+=['',f'제어 deadline 경고 합계: {sum(m["controller_deadline_warning_count"] for m in new)}. sim/wall 비율·jerk·정지·재출발·방향 반전은 [전체 CSV](diverse8/summary.csv)에 있다. 정체한 실패 실행의 낮은 jerk를 개선으로 해석하지 않는다.']
m1=next((m for m in new if m['goal_index']==7),None)
if m1:
 log=(r/'diverse8/7_other_room/launch.log').read_text();lines+=['',f'M1의 BackUp 실행 횟수는 {log.count("Running backup")}회였다. 최종 M1 성공만으로 복구 동작의 성공을 주장하지 않는다.']
lines+=['','## 보존한 비교와 실패','','| 실행 | Nav2 | GT 위치 m | GT 방향 ° | 판단 |','|---|---:|---:|---:|---|']
for name,label in [('control_m1','기존 범용 수정의 M1 단독 대조'),('segment_m1','새 SE(2) 구간 비용'),('tracking_m1_v2','정렬 가중치 48·짧은 추종 거리')]:
 m=json.loads((r/name/'metrics.json').read_text());lines.append(f'| [{label}]({name}/metrics.json) | {m["action_status"]} | {m["position_error_m"]:.3f} | {m["yaw_error_deg"]:.2f} | {"단독 성공, 반복 안정성 근거 아님" if name=="control_m1" else "실패, 채택하지 않음"} |')
lines+=['','첫 전체 8개 묶음의 M1은 Nav2=6, GT 1.162m/59.50°였다. 나머지 7개는 통과했다. [첫 8개 보고서](FIRST8_REPORT.md), [첫 8개 CSV](regression8/summary.csv). 이 결과를 최종 성공 실행으로 덮어쓰거나 대체하지 않았다.', '',
'`tracking_m1`은 주행 전 ros2 param dump 실패로 제외했다. `tracking_m1_v2`는 실제 주행 실패이며 별도로 보존했다. 각 최종 경로는 1회씩 검사했고, 수정 전후 전체 검증을 각각 수행했다. 요청하지 않은 5회 반복 캠페인은 하지 않았다.', '',
'## 원인 확인과 검증 수준','',
'첫 묶음 M1 실패 로그는 MPPI 후보 계산 실패 → BackUp 충돌 거부 → 전체 실패 순서였다. 실패 후 저장된 local/global costmap에서 현재 footprint의 native cost는 253(nonlethal)이었다. 이는 현재 자세의 정적 검사이며, 실제 실패 순간의 모든 동적 rollout이 안전했다는 뜻은 아니다. [native 검사](regression8/7_other_room/native_pose_costs.json).', '',
'저장된 실패 지점의 경로를 같은 규칙에 가깝게 crop하여 정지 상태로 재생했을 때, 첫 128개 경로 후보의 서로 다른 궤적은 8종에서 72종으로 늘었다. 두 방식 모두 정지 재생에서는 비충돌 후보가 존재했다. 따라서 원래 이동 중 실패를 완전히 재현한 실험이나 단일 원인 증명으로 해석하지 않는다. Gaussian batch도 실행마다 다르다. [재생 요약과 한계](failure-replay/summary.json).', '',
'후보 생성 단위 검사는 coupled connector·휠 제한·yaw wrap·빈 경로·다른 연결점의 제어 다양성을 확인했다. 실제 Nav2 PipelineSequence/RecoveryNode와 BT.CPP 기반 테스트는 막힌 후퇴 뒤 재계획 성공, 초기 계획 실패, 지속적인 제어 실패의 4회 상한을 검사했다. 높은 정렬 가중치의 실패 실행에서도 4회 후퇴 거부 후 종료를 관찰했다. [BT 테스트 결과](test-recovery-result.txt).', '',
'새 비용 함수 실험은 단위 검사를 통과했지만 M1에서 실패해 채택하지 않았다. 최종 설정에서는 사용하지 않는다. 원래 노트북 frozen-state 재생과 이번 실제 주행을 혼동하지 않는다.', '',
'## 동일 조건 검사와 한계','',
'[실제 설정 검사](diverse8/general_audit.json)는 각 실행의 원래 목표 하나·경유점 없음·중간 checker 없음·같은 BT·같은 공통 설정·동일 MPPI 라이브러리 해시·물리 제약 검사를 묶는다. 각 실행의 audit.json, controller_server.yaml, parameters.yaml, navigation_stages.json도 보존했다.', '',
'footprint `[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]`, padding=0, wheel cap 5rad/s·반경 0.0635m·wheelbase 0.35m·separation 0.6038m, 최종 30cm/10°를 유지했다. stock Smac Lattice 5cm/0.5m omni, reverse=true, smooth=false, max_iterations=-1, planning 80초를 유지했다. AMCL 0.01m/0.01rad 및 매 1sim초 nomotion update, whole 18F world와 목표 의자만 제거하는 조건도 동일하다. GT는 평가에만 사용했다.', '',
'이는 지정된 8개 경로의 단회 검증이다. 반복 성공률, 미지의 환경이나 모든 문에서의 안정성을 증명하지 않는다. M1 단독 성공 후 첫 전체 묶음 실패를 실제로 관찰했으므로 이 한계가 특히 중요하다. 노트북/실물 로봇 프로세스는 변경하지 않았고 git push도 하지 않았다.', '',
'## 재현과 산출물','',
'```bash','cd /home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment','distrobox enter cleany-ros-humble-cdi -- bash build.sh candidate_diverse','distrobox enter cleany-ros-humble-cdi -- python3 run_campaign.py \\','  --profile general --install candidate_diverse/install \\','  --config "$PWD/generalization/common.yaml" \\','  --bt "$PWD/generalization/recovery_replan.xml" \\','  --order 7,0,1,2,3,4,5,6 \\','  --output "$PWD/generalization/reproduce8" --domain-base 220','```','','기존 출력 디렉터리를 덮어쓰지 않는다. 동시 실행이 있다면 겹치지 않는 ROS domain을 선택한다. 실행기는 자신의 프로세스 그룹만 정리한다.', '',
'- [최종 CSV](diverse8/summary.csv), [최종 JSON](diverse8/summary.json)','- [8개 통합 GIF](eight-routes.gif), [8개 GT 경로 그림](eight-routes.png), [M1 GT 재생 GIF](diverse8/7_other_room/drive.gif)', '- 결과 표의 각 경로 이름을 누르면 해당 경로의 개별 GIF가 열린다. 통합 GIF는 출발 후 경과한 sim 시간을 맞춰 재생한다. [재생 정보](gifs.json).','- GIF는 기록된 Gazebo ground truth를 지도 위에 재생한 것이며 GUI 화면 녹화가 아니다.','- 각 실행에 bag/, candidate_costs.csv, snapshots/, contacts.jsonl, samples.csv, compute_timing.csv를 보존했다.','']
(r/'REPORT.md').write_text('\n'.join(lines))
