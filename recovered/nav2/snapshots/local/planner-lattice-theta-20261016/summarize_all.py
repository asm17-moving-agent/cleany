from pathlib import Path
import ast
import csv
import json
import math
import yaml

ROOT=Path(__file__).resolve().parent
PLANNERS=['lattice','theta']
NAMES=['straight','turn_arrival','chair_gap','far_heading','distant','lateral','narrow_seat','other_room']
rows=[];checks=[]
failure_details=[]
for i,name in enumerate(NAMES):
    contracts=[];binaries=[];configs=[]
    for planner in PLANNERS:
        folder=ROOT/planner/f'{i}_{name}'
        result=json.loads((folder/'result.json').read_text())
        metric=json.loads((folder/'metrics.json').read_text())
        contract=json.loads((folder/'comparison_contract.json').read_text())
        params=yaml.safe_load((folder/'parameters.yaml').read_text())
        contract['controller']['ros__parameters']['FollowPath'].pop('benchmark_timing_file',None)
        contracts.append(contract);binaries.append(result['loaded_mppi_libraries_sha256']);configs.append(params)
        compute=metric.get('compute_timing',{}).get('compute_us',{})
        smooth=result.get('smoothness',{})
        cmd=result.get('max_command_wheel_rad_s',{})
        plans=json.loads((folder/'plans.json').read_text())
        first=plans[0]['poses'] if plans else []
        row={'planner':planner,'goal_index':i,'goal_name':name,'nav2_status':result.get('action_status'),
             'gt_xy_cm':result.get('position_error_m',math.nan)*100,'gt_yaw_deg':result.get('yaw_error_deg'),
             'sim_s':result.get('navigation_sim_s'),'wall_s':result.get('navigation_wall_s'),
             'actual_rtf':result.get('achieved_rtf'),
             'compute_median_ms':compute['median']/1000 if 'median' in compute else None,
             'compute_p95_ms':compute['p95']/1000 if 'p95' in compute else None,
             'compute_p99_ms':compute['p99']/1000 if 'p99' in compute else None,
             'planner_compute_ms':None,'planner_compute_null_reason':'Planner-only wall compute was not instrumented',
             'linear_jerk_rms':smooth.get('linear_jerk_rms_m_s3'),'linear_jerk_p95':smooth.get('linear_jerk_p95_m_s3'),
             'angular_jerk_rms':smooth.get('angular_jerk_rms_rad_s3'),'angular_jerk_p95':smooth.get('angular_jerk_p95_rad_s3'),
             'stationary_s':smooth.get('stationary_s'),'motion_starts':smooth.get('motion_start_count'),
             'yaw_reversals':smooth.get('yaw_reversals'),'lateral_reversals':smooth.get('lateral_reversals'),
             'max_wheel_command_rad_s':cmd.get('/cmd_vel'),'max_wheel_joint_rad_s':metric.get('measured_joint_max_rad_s'),
             'non_floor_contact_records':metric.get('contacts',{}).get('non_floor_non_self_records'),
             'controller_deadline_warnings':result.get('controller_deadline_warning_count'),
             'first_plan_length_m':sum(math.dist(a[:2],b[:2]) for a,b in zip(first,first[1:])) if first else None,
             'plan_count':len(plans),'error':result.get('error')}
        rows.append(row)
        if result.get('action_status')!=4:
            log=(folder/'launch.log').read_text(errors='replace')
            excerpt=[line for line in log.splitlines() if any(s in line for s in
                ['no collision-free emitted rollout','Collision Ahead','backup failed','Goal failed','Failed to make progress'])]
            failure_details.append({'planner':planner,'goal':name,'final_gt':result.get('final_truth'),
                'goal_remaining_m':result.get('position_error_m'),'runtime_log_excerpt':excerpt[-20:],
                'root_cause':None,'root_cause_null_reason':'Observed failure symptoms only; no additional tuning or causal rerun was performed in this comparison'})
    equal={k:contracts[0][k]==contracts[1][k] for k in ['controller','bt_sha256','map_sha256','world_sha256','manual_waypoints','mppi_install','requested_rtf']}
    equal['loaded_mppi_binaries']=bool(binaries[0]) and bool(binaries[1]) and binaries[0]==binaries[1]
    for planner,params in zip(PLANNERS,configs):
        ctrl=params['controller_server']['ros__parameters'];f=ctrl['FollowPath']
        equal[planner+'_wheel5']=f['wheel_max_rad_s']==5.
        equal[planner+'_arrival30cm10deg']=ctrl['goal_checker']['xy_goal_tolerance']==.3 and ctrl['goal_checker']['yaw_goal_tolerance']==math.radians(10)
        for cm in ['local_costmap','global_costmap']:
            cfg=params[cm][cm]['ros__parameters']
            equal[f'{planner}_{cm}_physical_footprint']=ast.literal_eval(cfg['footprint'])==[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]] and cfg['footprint_padding']==0
    checks.append({'goal':name,'checks':equal})
audit={'paired_condition_checks':checks,'all_equal':all(all(c['checks'].values()) for c in checks)}
(ROOT/'condition-audit.json').write_text(json.dumps(audit,indent=2))
if not audit['all_equal']:raise RuntimeError('Paired conditions differ; inspect condition-audit.json before comparison')
with (ROOT/'comparison.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
(ROOT/'comparison.json').write_text(json.dumps({'runs':rows,'failures':failure_details,'condition_audit':audit,'replicates_per_route':1,'scope':'Actual Gazebo closed-loop runs; GT map replay GIF, not GUI recording; no composite score or pass threshold.'},indent=2))
lines=['# Lattice / Theta* + 동일 MPPI: 공통 8경로 주행', '',
       '두 플래너를 공통 8개 목표에 각각 1회 새로 실행했다. 수동 경유점 없이 HOME에서 원래 목표 하나만 전송했다. 각 주행은 독립 world·ROS domain·Gazebo partition을 사용했다.', '',
       f"이번 묶음의 Nav2 도착은 Lattice {sum(r['nav2_status']==4 for r in rows if r['planner']=='lattice')}/8, Theta* {sum(r['nav2_status']==4 for r in rows if r['planner']=='theta')}/8이다. Nav2가 도착한 주행들의 GT 오차는 모두 기존 30cm/10도 안이었다. Theta*의 다른 방 이동은 경로 생성 후 주행 도중 종료됐으며, MPPI 출력 rollout 충돌 거부와 후퇴 충돌 거부 로그가 기록됐다. 반복 성공률이나 플래너 전체의 확정 우열로 해석하지 않는다.", '',
       'MPPI는 desktop-cleany 후속 개선의 candidate_diverse 버전이다. 동일한 컨트롤러·비용·출력 충돌 검사·진행량 감지·복구 BT를 유지했다. 두 플래너의 지도/월드/컨트롤러/실제 라이브러리 SHA256 일치 검사는 condition-audit.json에 기록했다. 플래너 자체는 stock Smac Lattice(기존 80초·5cm omni primitives)와 stock Theta* 기본 설정이다. 두 플래너의 native 경로 방향 정책은 변경하지 않았다.', '',
       '휠 명령 5rad/s, 실제 footprint 47.7×65.48cm/padding0, 최종 30cm/10도를 유지했다. 목적지 의자는 기존 잠금 파일에 지정된 것만 제거했다. AMCL을 사용하고 GT는 관측에만 사용했다.', '',
       '이전 Gazebo 완화 실험과 동일하게 외부 pan 정렬·CollisionMonitor·3D guard는 shadow 출력이며 실제 출력은 Nav2→휠 결합 제한→Gazebo다. MPPI 내부 footprint/출력 궤적 충돌 검사는 유지했다. 초기 Lattice 회전 도착 실행은 목표 전송 전 ROS CLI 파라미터 덤프 대기로 중단됐으며 startup-failed 디렉터리에 보존했다. 계측을 동일 서비스의 직접 조회로 바꾼 뒤 재시작했고 성능 표에는 실제 목표를 보낸 주행만 포함했다.', '',
       '## GIF', '',
       '[2행 × 8열, 위 Lattice / 아래 Theta*, 2배속](media/lattice-theta-2rows-8cols-2x.gif)', '',
       '실제 Gazebo GT를 해당 주행 지도 위에서 재생한 GIF이며 GUI 화면 녹화가 아니다. 차체 실제 크기·방향과 이동 궤적을 표시했다. 0.4sim초마다 200ms를 재생하므로 2배속이다. 각 열은 동일 목표와 동일 배율이며 완료 후 최종 자세를 유지한다. 실패/정지 구간은 삭제하지 않았다.', '',
       '## 경로별 관측 결과', '',
       '| 경로 | 플래너 | Nav2 status | GT cm / ° | sim s | 실제 RTF | MPPI compute p95 ms | 정지 s | joint 최대 rad/s | 접촉 기록 |',
       '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
def fmt(x,n=2):return 'null' if x is None else f'{x:.{n}f}'
for r in rows:
    lines.append(f"| {r['goal_name']} | {r['planner']} | {r['nav2_status']} | {fmt(r['gt_xy_cm'])} / {fmt(r['gt_yaw_deg'])} | {fmt(r['sim_s'])} | {fmt(r['actual_rtf'],3)} | {fmt(r['compute_p95_ms'])} | {fmt(r['stationary_s'])} | {fmt(r['max_wheel_joint_rad_s'],3)} | {r['non_floor_contact_records']} |")
lines+=['', 'Nav2 4=SUCCEEDED, 5=CANCELED, 6=ABORTED. GT 오차와 Nav2 status를 별도로 표시한다. compute는 TimedController 내부 MPPI 계산 wall 시간이며 planner 계산 시간이 아니다. planner 전용 계산 시간은 이번 계측에서 null(별도 계측 없음)이다. 접촉은 비바닥·비자체 센서 기록 수이며 0이 독립적인 비침투 증명은 아니다. 휠 명령 제한과 물리 joint 응답은 별도 수치다.', '',
        f"관측된 최대 휠 명령은 {max(r['max_wheel_command_rad_s'] for r in rows):.6f}rad/s, 실제 joint 응답 최대는 {max(r['max_wheel_joint_rad_s'] for r in rows):.3f}rad/s였다. 따라서 5rad/s 설정을 실제 관절 응답의 절대 상한으로 주장하지 않는다. 제어주기 누락 경고는 전체 {sum(r['controller_deadline_warnings'] for r in rows)}회이며, 병렬 실행 부하와 계측을 포함한 결과다.", '',
        '각 경로 1회 관측이므로 반복 성공률이나 통계적 우위를 주장하지 않는다. 처음에는 Gazebo 2개, 사용자 추가 병렬 요청 후에는 최대 4개를 동시에 실행했다. 오래 걸리는 M1 두 주행을 별도 worker로 먼저 예약했으며, 기존 순차 worker는 해당 폴더를 만나면 중복 실행하지 않는다. 실제 RTF와 연산시간은 당시 공유 부하의 영향을 받으므로 플래너 고유 연산 성능 순위로 해석하지 않는다. 요청 RTF는 2이나 실제 RTF는 위 관측값을 사용한다. 실패한 조기 종료의 시간·낮은 jerk는 개선으로 해석하지 않는다. 점수나 합격 기준은 추가하지 않았다.', '',
        '전체 정확도·시간·연산 median/p95/p99·jerk·정지·반전·휠·접촉 수치는 [CSV](comparison.csv)와 [JSON](comparison.json)에 있다.', '', '개별 경로 GIF:', '']
lines += [f'- [{i+1}. {n}](media/{i+1:02d}-{n}-pair-2x.gif)' for i,n in enumerate(NAMES)]
if failure_details:
    lines+=['', '## 종료한 실패 주행', '']
    for failure in failure_details:
        lines.append(f"- {failure['planner']} / {failure['goal']}: 최종 GT {failure['final_gt']}, 목표까지 {failure['goal_remaining_m']:.2f}m. 실제 로그의 오류/복구 이벤트는 comparison.json의 failures에 보존했다. 경로 생성 이후 주행 실패와 경로 탐색 실패를 혼동하지 않는다. 원인 확정을 위한 추가 튜닝/재실험은 하지 않았다.")
lines += ['', '원격 원자료: /home/donglee/workspace/cleany/planner-lattice-theta-20261016. 기존 소스·설정·산출물은 보존했고 git push/물리 로봇 검증은 하지 않았다.']
(ROOT/'REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'runs':len(rows),'nav2_succeeded':{p:sum(r['nav2_status']==4 for r in rows if r['planner']==p) for p in PLANNERS},'condition_audit':audit['all_equal']}))
