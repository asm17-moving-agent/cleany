from pathlib import Path
import json,csv,math
r=Path(__file__).resolve().parent/'fault_recovery'
result=json.loads((r/'result.json').read_text());diags=json.loads((r/'diagnostics.json').read_text());hold=[v for v in diags if v.get('reason')=='TEST_COMMAND_HOLD']
with (r/'compute_timing.csv').open() as f:timings=list(csv.DictReader(f))
failed=[v for v in timings if v['success']=='0']
with (r/'velocity.csv').open() as f:vel=[list(map(float,v)) for v in csv.reader(f)]
interval=[min(v['sim_s'] for v in hold),max(v['sim_s'] for v in hold)];steady=[v for v in vel if interval[0]+.5<=v[0]<=interval[1]]
plans=json.loads((r/'plans.json').read_text());e=json.loads((r/'recovery_evidence.json').read_text())
out={'injection':'Independent Gazebo only: zero emitted commands in absolute sim interval[12,28)','observed_hold_interval_sim_s':interval,'held_command_count':len(hold),'held_outputs_all_zero':all(all(abs(x)<1e-12 for x in v['output']) for v in hold),'navigation_start_sim_s':result['navigation_sim_start_s'],'first_failed_compute_sim_s':int(failed[0]['sim_time_ns'])/1e9 if failed else None,'plans_published_during_hold':sum(interval[0]<=v['sim_s']<=interval[1] for v in plans),'steady_hold_max_gt_linear_m_s':max((math.hypot(v[1],v[2]) for v in steady),default=None),'route_timeout_log_count':e['route_timeout_log_count'],'backup_completed_log_count':e['backup_completed_log_count'],'action_status':result['action_status'],'gt_xy_m':result['position_error_m'],'gt_yaw_deg':result['yaw_error_deg'],'scope':'Diagnostic fault-injection run, excluded from normal navigation performance comparisons'}
(r/'fault_evidence.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
