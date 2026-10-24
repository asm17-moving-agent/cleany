"""Generate Korean report from preserved metrics; unfinished results stay explicit."""
from pathlib import Path
import json
R=Path(__file__).resolve().parent

def read(path):
 p=R/path
 return json.loads(p.read_text()) if p.exists() else None

def number(v,d=3):return '—' if v is None else f'{v:.{d}f}'
def table(runs):
 s='| 실행 | Nav2 | GT XY(m) | GT yaw(°) | sim(s) | wall(s) | RTF | GT 기준 포함 성공 |\n|---|---:|---:|---:|---:|---:|---:|---|\n'
 for name,m in runs:
  if m is None:s+=f'| {name} | 미완료 | — | — | — | — | — | 미완료 |\n';continue
  s+=f'| [{name}]({name}/metrics.json) | {m.get("action_status")} | {number(m.get("position_error_m"))} | {number(m.get("yaw_error_deg"),2)} | {number(m.get("navigation_sim_s"),2)} | {number(m.get("navigation_wall_s"),2)} | {number(m.get("achieved_rtf"))} | {"성공" if m.get("success") else "실패"} |\n'
 return s
def quality_table(runs):
 s='| 실행 | compute p50/p95/p99 (ms) | 휠 명령/관절 최대(rad/s) | 정지(s) | 출발/횡반전/yaw반전 | 선형/각 jerk RMS | 비바닥 접촉 기록 |\n|---|---|---|---:|---|---|---:|\n'
 for name,m in runs:
  if not m:continue
  t=m.get('compute_timing',{}).get('compute_us',{});v=m.get('smoothness') or {};w=m.get('max_command_wheel_rad_s') or {}
  s+='| '+name+' | '+' / '.join(number(t.get(k,0)/1000,2) for k in ['median','p95','p99'])+' | '+number(w.get('/cmd_vel'))+' / '+number(m.get('measured_joint_max_rad_s'))+' | '+number(v.get('stationary_s'),2)+' | '+' / '.join(str(v.get(k,'—')) for k in ['motion_start_count','lateral_reversals','yaw_reversals'])+' | '+number(v.get('linear_jerk_rms_m_s3'))+' / '+number(v.get('angular_jerk_rms_rad_s3'))+' | '+str(m.get('contacts',{}).get('non_floor_non_self_records','—'))+' |\n'
 return s

before=[(k,read(k+'/metrics.json')) for k in ['baseline_m1_v3','pathseed_m1_v2','pathseed_m1_v3','prealign_m1_v1']]
goals=json.loads((R.parents[1]/'controller-comparison-8goals-20261014/common/goals.locked.json').read_text())['goals']
campaign=[('regression8/'+str(g['index'])+'_'+g['id'],read('regression8/'+str(g['index'])+'_'+g['id']+'/metrics.json')) for g in goals]
finished=sum(m is not None for _,m in campaign);passed=sum(bool(m and m['success']) for _,m in campaign)
status='완료' if (R/'FINALIZED').exists() else '진행 중'
s=f'''# Smac Lattice + MPPI M1 문 통과 진단 및 개선

상태: **{status}**. 최종 HOME→M1 주행은 **Nav2 status4, GT5.08cm/8.40°**로 문 통과와 도착을 확인했다. **138.956sim초 /326.595wall초, RTF0.42547**, 측정 휠 최대4.87013rad/s, 비바닥 접촉 기록0이다. 최초 공통8목표는{passed}/{finished}개 성공이며, 실패했던 회전 도착과M1은 수정 후 개별 재검증에서 성공했다. 최초 실패와 개발 중 재실패 기록은 모두 남겼다. 전체8개를 수정 후 다시 반복하거나 추가5회 반복하지 않았다.

## 최종 수정의 M1 검증

{table([('portal_hold_m1',read('portal_hold_m1/metrics.json'))])}

최초 개선의 긴 문 통과 구간은 8목표 시험에서 재실패했다. 이를 보존한 뒤 문 구간을0.30m 단위의 짧은 목표로 나눴다. 좌표는 실제5cm lattice cell center이며 중간checker2cm/2°, goal rotation XY lock .010/.015m다. 전체1845개의 ±2cm/±2° 자세에서 전역·지역 native 충돌 판정0을 확인했다. [최종 M1 경유점](portal_approaches_v3.yaml), [최종 M1 튜닝](prealign_v2.yaml), [검사](native/portal-clearance-v2.json). 최종 도착30cm/10°, wheel5rad/s, footprint/padding 및 내부 충돌 검사는 그대로다. 짧은 목표v2도 주기적 재계획이 .825→.525 후진·회전 우회→.925의 loop로 바뀌면서 실패했다. 이 시점 마지막1000개 후보의 CostCritic은 모두0이고 끝점은 모두 현재보다 뒤쪽이었다. 따라서 최종v3는 문 안의 짧은 구간에만 [초기 Smac 경로 유지 BT](stage_hold_path.xml)를 사용하고 충돌/진행량 실패 때 다시 계획한다. 접근과 최종 목표의 주기적 재계획은 유지한다. [v2 실패 후보](replan_fix_m1/candidate_summary.json). 단일 후속 주행으로 반복 성공률을 주장하지 않는다.

## 초기 개선의 실제 M1 도착

`prealign_m1_v1`: Nav2 status 4, GT 위치 오차 **0.274276m**, yaw **7.62939°**, **126.769sim초 / 299.954wall초**, RTF **0.42263**. 최종 허용치 0.30m/10° 이내다. 시작은 HOME `[-1.865,-4.705,1.5708]`, 최종 목표는 기존 M1 `[-16.775,1.775,1.570796]`이다. 명령 및 측정 관절 휠 최대 **4.90846rad/s**, 기록된 비바닥·비자체 접촉은 **0**이다.

문 밖 `[-17.125,0,pi]` 정렬 → 문 안 `[-17.125,2,pi]` 출구 → 원래 M1 목표의 세 단계가 모두 성공했다. 실제 문 구간에서는 Smac이 선택한 비스듬한 중간 자세도 사용됐다. yaw=180°를 통과 내내 강제로 고정했다고 주장하지 않는다. 최종 접근의 재계획 실패 후 native collision 검사를 사용하는 BackUp이 실행됐고 GT로 **0.35537m 후퇴**, 재계획 후 도착했다. [단계별 기록](prealign_m1_v1/navigation_stages.json), [후퇴 증거](prealign_m1_v1/recovery_evidence.json), [실행 제약 검사](prealign_m1_v1/audit.json).

## 원인: 확인된 것과 남은 추정

1. **기하학적으로 결합 이동이 필요했다.** 전달받은 원래 정체 지도/경로를 현재 호스트의 native `FootprintCollisionChecker`로 다시 검사했다. 현재 pose 비용253은 lethal이 아니다. 실제 경로 29개 및 연결 보간244개는 충돌0인 반면, 제자리 회전은 8/31, 현재 yaw 고정 이동은 35/244 충돌이었다. 단순 회전 또는 평행 이동만 시도해서는 유효 경로를 대체할 수 없다. [원시 검사](native/native-maneuver-check-result.json).
2. **고정 상태 재생에서 초기 Gaussian yaw 후보 범위가 부족했다.** 필요한 yaw 변화는 +19.33°인데 첫 1000개 Gaussian 후보의 범위는 −6.69~+5.54°였다. 기존 critic을 유지한 연속 SE(2) 경로 후보는 +19.33°를 포함하고 최저 critic 합계가 16.704→9.135로 내려갔다. PathFollow 14.774→5.808, Cost 1.466→2.015, PathAlign 0→0.467, Twirling 0.464→0.844였다. 즉 기존 critic이 모든 유효 결합 동작을 거부한다는 해석은 이 시험과 맞지 않는다. [전체 재생 결과](native/replay/diagnosis.json), [그림](native/diagnosis.png).
3. **PathAlign gate와 늦은 자세 정렬을 관찰했다.** 초기 furthest가 offset2보다 작으면 PathAlign 비용이 0이었다. 실제 `pathseed_m1_v3`도 복도는 통과했지만 약150° 자세로 M1 문턱에 진입한 뒤 실패했다. 문 밖 회전 가능 영역과 yaw=180° 통과 경로를 전역·지역 native footprint 검사로 확인해 사전 정렬/출구 경유점을 추가했다. [문 형상 검사](native/portal-clearance.json), [실제 후보 요약](pathseed_m1_v3/candidate_summary.json).
4. **평균/필터 출력은 별도 검증이 필요했다.** 이미 평가된 후보들의 weighted mean과 Savitzky–Golay 결과에서 충돌 rollout이 실제로 검출됐다. 최종 출력에 dense native 검사를 추가하고 비용순 유효 후보로 교체한 로그를 저장했다. 모든 후보가 무효면 예외로 recovery에 넘긴다. 유효 후보의 평균이 유효하다는 가정을 제거한 것이다.
5. **무한 대기를 제거하고 실제 복구를 확인했다.** route 진행량12초와 bounded recovery를 적용했다. M1 성공 주행에서 후퇴/재계획이 실제 작동했다. 진행량 timeout의 별도 명령 정지 시험 결과는 아래 절에 명시한다.

현재 BackUp은 몸체 -X 방향이다. 문에서 횡이동 중 이 방향이 막히면 충돌 검사에 의해 거부되고 작업은 실패로 종료된다. 모든 문턱 자세에서 탈출하는 범용 복구기를 구현했다고 주장하지 않는다. 자유 공간의 실제 후퇴/재계획과 문 밖 사전 정렬은 별도로 검증했다.

원래 노트북 optimizer의 warm nominal 시퀀스는 보존되지 않았다. 고정 재생은 속도0, 고정 위치, 고정 Gaussian batch를 사용했고 실제 주행은 noise를 재생성한다. 따라서 후보 부족은 통제된 재생에서 확인된 원인이며 원래 장시간 정체의 모든 내부 상태를 동일 재현했다는 뜻은 아니다. PathAlign/Twirling 한 항목만을 단독 원인으로 확정하지 않는다. 여러 변경을 함께 적용했으므로 각 변경의 독립 효과나 통계적 성공률도 주장하지 않는다.

`offset_from_furthest=0`의 기존 SIGSEGV는 별개 문제다. 소스의 `for (i=1; i != path_segments_count; i++)`는 count0에서 유효하지 않은 접근을 할 수 있다. 모든 실제 실행은 offset2를 유지했으며 0으로 라이브 재시험하지 않았다.

## 적용 변경과 유지 조건

- 연속 경로 SE(2) 후보128/1000, 최대 제안 속도0.25m/s, 나머지 Gaussian 후보와 함께 모든 기존 critic 평가. PathAlign weight12, offset2, path orientation 사용.
- 평균/필터 이후 실제 footprint rollout을 최대1cm/1° 간격으로 검사. lethal≥254 및 map 밖 판정은 거부한다. 비용253을 충돌로 바꾸거나 collision critic을 끄지 않는다.
- 같은 최종 목표의 참조 경로 arc 진행량0.08m/12sim초. 반복 재계획만으로 시간을 초기화하지 않는다. 최종 XY 영역 안 yaw 개선은 진행으로 인정한다.
- controller failure tolerance0.5초, 보조 SimpleProgressChecker0.10m/20초. 접근/최종 목표는5초마다 stock Smac 재계획, 문 안의 짧은 구간은 초기 경로를 유지하고 실패 때 재계획. 실패 시 BackUp0.35m/0.10m/s/8초 제한, 최대4회. 전체300sim초 또는1800wall초 제한.
- 최종 M1의 중간 checker는2cm/2°(초기개선은3.5cm/3°). 최종 checker는 **30cm/10° 그대로**. goal rotation XY lock은 최종M1 .010/.015m, 일반 프로필 .025/.030m.
- wheel5rad/s coupled projection, radius0.0635m, wheelbase0.35m, separation0.6038m. body yaw 최대5rad/s 설정이 아니다.
- footprint `[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]`, padding0. 전역/지역 모두 일치 확인.
- 새18F 전체 world, 목표 의자만 제거. M1은 `facility_chair_M14`. canonical 모델 집합과 geometry 비교 및 실제 server scene 로드 확인.
- stock Smac Lattice, 5cm/0.5m omni primitive, reverse=true, smooth=false, max_iterations=-1, planning80초. StraightFirst 사용하지 않음.
- AMCL update_min_d/a=.01/.01, 1sim초 nomotion update. GT는 관찰만 사용하며 주행 위치로 주입하지 않음.
- 노트북의 기존 relaxed 실험과 동일하게 외부 pan/2D/3D guard는 shadow 출력이고 실제 출력은 Nav2→wheel projection→Gazebo다. 이번 개선을 위해 외부 또는 내부 충돌 검사를 추가로 끄지 않았다.

[최종 실제 적용 파라미터](portal_hold_m1/parameters.yaml), [최종 설정 모음](final_config/), [M1 튜닝](prealign_v2.yaml), [경유점](portal_approaches_v3.yaml), [일반 BT](recovery_keep_path.xml), [문 구간 BT](stage_hold_path.xml), [소스 패치](candidate-v2.patch).

## 동일 조건 M1 전후 및 중간 실패

환경·HOME·월드·제약·측정 정의를 유지했다. 기준선은 노트북 최적화 MPPI에 동일 telemetry만 추가한 것이다. 데스크톱 기준선은 문 앞까지 도달하지 못하고 앞 복도에서 실패했다. 원래 문 앞 정체의 폐루프 재현 성공으로 잘못 표시하지 않는다. 초기화 실패 실행들은 아래 성능 표에서 제외했다.

{table(before)}

기준선은 모든 후보 충돌 후 재계획 시작점 lethal로 종료했다. proposals64만 적용한 v2는 복도 곡선 정렬이 부족했고 후퇴도 충돌로 거부됐다. proposals128/PathAlign12/출력 검증의 v3는 문 앞까지 도달했지만 실패했다. 최종 사전 정렬/출구 경유점 조합에서 M1 성공을 확인했다.

## 공통 8개 목표 — 각 1회

매 실행 새 world, 독립 ROS domain220..227 및 UUID Gazebo partition을 사용한다. 각 목표별 지정 의자만 제거한다. 추가5회 반복은 없다. 아래 표의 성공은 Nav2 status4와 GT≤0.30m/10°를 동시에 만족한다.

{table(campaign)}

[최초8 JSON](regression8/summary.json), [최초8 CSV](regression8/summary.csv), [진단·최종검증 통합 JSON](RESULTS.json), [통합 CSV](RESULTS.csv), 각 실행의 `launch.log`, `snapshots/`, `candidate_costs.csv`, `parameters.yaml`에 원시 증거가 있다. 실패 원인/추가 진단은 `regression8/failure_analysis.json`에 기록한다.

## 연산·주행 품질·접촉 해석

TimedController의 steady wall clock으로 inner compute를 측정한다. telemetry와 출력 검증 비용을 포함하고 simulator/planner는 제외한다. 실패 호출도 보존한다. `metrics.json`과 CSV에 median/p95/p99/max, GT jerk RMS/p95, 정지 시간/출발 횟수/횡·yaw 반전, sim/wall/RTF, 명령 및 joint wheel 최대를 함께 기록한다.

{quality_table(before)}

{quality_table(campaign)}

{quality_table([(k,read(k+'/metrics.json')) for k in ['replan_fix_turn_arrival','portal_hold_m1']])}

최종 M1의 정지 시간은8.65초, 출발13회, 선형 jerk RMS1.673m/s³였다. 짧은 중간 목표를 추가하면서 초기1회성공 주행(정지1.70초,출발5회,선형jerk1.363m/s³)보다 정지·출발과 선형jerk가 늘었다. 도착 검증 결과와 부드러움 수치를 분리해 해석한다.

정지 정의는 |v_xy|<0.01m/s AND |wz|<0.02rad/s이다. 원래 probe의 공통 GT 평활·차분 정의(50ms 재표본,5샘플 이동평균,gradient2회,양끝5샘플 제외)를 사용했다. yaw/횡 반전 deadband는 각각0.02rad/s와0.01m/s다. **정체나 조기 실패로 줄어든 jerk/시간을 개선으로 해석하지 않는다.** 기준선이 목적지까지 달리지 못했으므로 end-to-end 시간·jerk의 우열이나 CPU 성능 개선을 주장하지 않는다. 확률적 MPPI/AMCL의 단일 실행이고 런타임 부하가 완전히 고정되지 않아 RTF와 compute는 관측치다.

모든6개 collision link에 observer를 두었다. 바닥 접촉은 4개 휠 양성 대조로 확인하고 비바닥/비자체 접촉은 전부 보존한다. body/head는 접촉 발생 시 publish하는 센서이므로 기록0을 독립적인 전신 비관통 증명으로 해석하지 않는다. 중간 실패 v3에서는 명령5rad/s 이하였지만 joint 응답 최대5.286rad/s가 관측됐다. 이 순간 물리 응답과 command 제한을 구분한다. 최종 M1 성공 실행은 명령/joint 모두4.90846rad/s였다.

## 재계획 경합의 후속 수정

회전 도착(1)은 controller의 도착 완료 뒤 주기적 planner abort가 recovery를 발동해 Nav2 status6으로 끝났다. GT가 기준 안이어도 실패로 남겼다. 새 BT는 초기 계획 성공을 필수로 요구하고, 주기적 재계획 결과를 별도 blackboard에 받아 성공한 경로만 현재 경로에 복사한다. 단순 ForceSuccess만 씌우면 충분하지 않다. Nav2 1.1.20의 ComputePathToPose는 abort/cancel/halt 때 출력 경로를 비우기 때문이다. [Nav2 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp), [typed blackboard 복사 구현](https://raw.githubusercontent.com/BehaviorTree/BehaviorTree.CPP/3.8.7/include/behaviortree_cpp_v3/actions/set_blackboard_node.h).

{table([(k,read(k+'/metrics.json')) for k in ['replan_fix_turn_arrival','replan_fix_m1','portal_hold_m1']])}

[수정 BT](recovery_keep_path.xml), [실제 Nav2 BT 라이브러리 회귀 시험](native/test-replan-tree-result.txt). 이 후속 시험은 발견한 회귀의 수정 확인이다. 최초 8목표 표를 덮어쓰지 않으며, 수정 BT로 나머지6목표 전체를 반복했다고 주장하지 않는다.

## 복구 timeout 시험

Gazebo 출력 명령을 sim12..28초 동안0으로 유지했다. 실제 정지 관찰 구간12.001..27.974초,618개 출력 모두0, 정지 안정 구간 GT 선속도 최대0m/s였다. 그동안 계획5개가 publish됐지만 최초 실패 compute는 sim23.499초(주행 시작11.312초 이후12.187초)로 무한 연장되지 않았다. route timeout 뒤 명령 해제 후 GT0.35402m 후퇴, 재계획 후 Nav2 status4/GT0.24902m/0.78317°로 도착했다. 이 주입 시험은 정상 성능 비교에서 제외한다. [정지·timeout 증거](fault_recovery/fault_evidence.json), [후퇴 증거](fault_recovery/recovery_evidence.json), [측정](fault_recovery/metrics.json).

## 재현·산출물·보존

실행 명령, 환경 source 순서, 개별 파일 의미는 [README](README.md)에 있다. [최종 M1 주행 GIF](portal_hold_m1/drive.gif) 및 [초기 성공 GIF](prealign_m1_v1/drive.gif)는 실제 Gazebo GT 기록을 실제 지도에 재생한 것이며 GUI 녹화가 아니다. [원인 비교 그림](native/diagnosis.png), [후퇴 기록](prealign_m1_v1/recovery_evidence.json), [초기 증거](provenance/), [작업 기록](WORKLOG.md)을 함께 보존한다.

물리 로봇/노트북 프로세스를 변경하지 않았다. 사용자가 Tailscale SSH 인증을 완료한 뒤 노트북 소스·설정만 읽어 복사했다. 이 작업은 dirty 메인 체크아웃을 편집하거나 커밋하지 않았고 기존 실험 산출물을 덮어쓰지 않았다. git push도 하지 않았다. 완료 점검에서는 메인HEAD가 별도 checkpoint 커밋57db456으로 바뀐 상태를 관찰했으며, 이 실험에 사용한 비교용 소스HEAD b3ca816은 동일하다. 초기/최종 체크아웃 상태를 provenance에 기록했다. 실행 소스는 이 디렉터리의 독립 복사본이며 런타임 실제 로드 library SHA256을 각 result.json에 기록했다. NVIDIA RTX5070의 실제 Ogre2 renderer를 확인했다. 호스트 NVML 버전 불일치는 기존 컨테이너 격리 드라이버를 사용해 우회했으며 시스템 드라이버를 바꾸지 않았다.
'''
(R/'REPORT.md').write_text(s)
print({'report':str(R/'REPORT.md'),'status':status,'campaign_finished':finished,'campaign_successes':passed})
