#!/usr/bin/env python3
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent

def table(rows,cols):
 def v(x):return 'null' if x is None else f'{x:.3f}' if isinstance(x,float) else str(x)
 return '\n'.join(['| '+' | '.join(label for key,label in cols)+' |','| '+' | '.join('---' for _ in cols)+' |']+['| '+' | '.join(v(r.get(key)) for key,label in cols)+' |' for r in rows])

def main():
 p=ROOT/'results.json';rows=json.loads(p.read_text()) if p.exists() else [];rows=sorted(rows,key=lambda x:x['goal_index'])
 p=ROOT/'common_snapshot/goals.locked.json';goals=json.loads(p.read_text())['goals'] if p.exists() else []
 text='# MPPI Omni — 18F facility 8개 목표 — 2026-10-14 KST\n\n'
 text+='각 목표 fresh world/home에서1회씩 실행했다. Nav2 status와 GT 오차는 별도 관측이며 합격 점수나 통합 순위를 계산하지 않는다. 이전5개 목표/15cm/옛 study-cafe 실험을 이번 결과에 혼합하지 않는다. Gazebo 전용이고 물리 검증은 없다.\n\n'
 text+='## 공통 조건과 잠금\n\n'
 text+='시작 map/world pose `[-1.865,-4.705,1.5708]`, 도착 XY .30m/yaw10deg, wheel 최대5rad/s, controller20Hz/model_dt=.05, RTF요청1, hard1200sim/1800wall이다. StraightFirstPlanner 및 actual footprint/padding/internal collision을 유지한다. 공통 DWB 담당이 footprint+planner/scene을 사전 검증한 READY와 locked JSON snapshot만 캠페인에 사용한다. 공유 common을 수정하지 않았다. 목표 의자만 제거하고 나머지 의자/형상을 canonical hash로 확인했다. URI는 소유 snapshot fixture에 materialize하고 실제 server scene/로그를 남긴다.\n\n'
 if goals:text+=table([{**g,'pose':g['pose_map_world']} for g in goals],[('index','index'),('id','목표'),('pose','잠금 map/world pose'),('remove_chair','제거 의자'),('coordinate_change','좌표 조정')])+'\n\n'
 else:text+='공통 READY 잠금 전 준비 단계다. 주행 goal은 아직 보내지 않았다.\n\n'
 text+='## 최종 목표별 관측\n\n'
 if rows:
  for cols in [
   [('goal_name','목표'),('outcome','결과'),('action_status','Nav2 status'),('position_error_cm','GT XY cm'),('yaw_error_deg','GT yaw deg'),('sim_s','sim s'),('wall_s','wall s'),('rtf','실제 RTF')],
   [('goal_name','목표'),('compute_median_ms','compute median ms'),('compute_p95_ms','p95'),('compute_p99_ms','p99'),('compute_exception_calls','예외'),('compute_over_50ms','50ms 초과'),('controller_cpu_one_core_pct','CPU one-core %'),('controller_rss_p95_mb','RSS p95 MiB'),('command_sim_interval_mean_s','command sim mean s'),('command_sim_interval_p95_s','p95 s')],
   [('goal_name','목표'),('compute_call_sim_period_median_s','call sim median s'),('compute_call_sim_period_p95_s','p95'),('compute_call_sim_period_p99_s','p99'),('compute_call_wall_period_median_s','call wall median s'),('compute_call_wall_period_p95_s','p95'),('compute_call_wall_period_p99_s','p99'),('nav2_cmd_vel_wall_period_mean_s','command wall mean s'),('nav2_cmd_vel_wall_period_p95_s','p95 s')],
   [('goal_name','목표'),('linear_jerk_rms_m_s3','선형 jerk RMS'),('linear_jerk_p95_m_s3','p95'),('angular_jerk_rms_rad_s3','각 jerk RMS'),('angular_jerk_p95_rad_s3','p95'),('full_stationary_s','전체 정지 s'),('full_motion_start_count','재출발'),('yaw_reversals','yaw 반전'),('lateral_reversals','vy 반전')],
   [('goal_name','목표'),('max_nav2_cmd_vel_wheel_rad_s','controller wheel rad/s'),('max_cmd_vel_wheel_rad_s','최종 command wheel'),('max_measured_wheel_rad_s','joint wheel'),('environment_contact_samples','환경 contact samples'),('environment_contact_episodes','episodes'),('contact_ground_samples','ground heartbeat samples'),('contact_unknown_name_samples','이름 미확인'),('environment_contact_max_depth_m','max depth m')]
  ]:text+=table(rows,cols)+'\n\n'
  text+='관측한 실패와 경계:\n\n'
  for x in rows:
   notes=[]
   if x.get('action_status')!=4:
    notes.append(f"Nav2 status={x.get('action_status')}, outcome={x.get('outcome')}")
    if x.get('error'):notes.append(x['error'])
   if x.get('position_error_cm') is not None and x['position_error_cm']>30:notes.append('GT XY가30cm 밖')
   if x.get('yaw_error_deg') is not None and x['yaw_error_deg']>10:notes.append('GT yaw가10deg 밖')
   if x.get('environment_contact_samples'):notes.append(f"환경 접촉 관측 {x['environment_contact_samples']} samples")
   if x.get('environment_contact_samples') is None:notes.append('환경 접촉 null: '+str(x.get('environment_contact_null_reason')))
   if notes:text+='- '+x['goal_name']+': '+ '; '.join(notes)+'. raw log/command_reason_timeline.json 참조.\n'
  text+='\n'
  boundary_path=ROOT/'boundary_evidence.json'
  if boundary_path.exists():
   evidence=json.loads(boundary_path.read_text())
   text+='## GT/TF 경계와 다른 방 진단\n\n'
   for x in evidence['goals']:
    if x['goal_index']==6:
     o=x['last_sampled_tf_gt']
     if o:text+=f"좁은 좌석의 마지막 navigation 표본(sim={o['sim_s']:.3f}, 종료 전 {o['age_before_navigation_end_s']:.3f}s)은 TF 목표 오차 {o['tf_goal_error_m']*100:.2f}cm, GT 목표 오차 {o['gt_goal_error_m']*100:.2f}cm, TF–GT 거리 {o['tf_gt_separation_m']*100:.2f}cm다. Nav2 status4와 GT30cm 초과를 별도 기록한다. 이 비동기 표본만으로 localization의 원인을 확정하지 않는다.\n\n"
    if x['goal_index']==7:
     text+=f"다른 방의 최종 status는 {x['action_status']}, timeout={x['timeout']}, error={x['error']}다. static preflight의 경로 존재가 runtime depth/memory costmap과 BT 경로 처리를 인증하지는 않는다.\n\n"
     for key,d in x['log_diagnostics'].items():
      if d['count']:text+=f"- `{key}`: {d['count']}회. 최초: `{d['first']}`\n"
     text+='\n이 로그는 관측한 오류 연쇄이며 정체의 단일 원인 증명이 아니다. shadow 가드 STOP과 실제 actuator 차단을 구분해 command_reason_timeline.json/commands.csv/bag를 확인한다. 정체 구간의 낮은 jerk를 성능 개선으로 해석하지 않는다.\n\n'
   text+=f"계측 runner SHA, 공통 snapshot/source {evidence['common_snapshot_sha256_verified_count']}개 파일 SHA 및 모든8개 bag의 SQLite quick_check/message count를 확인했다. `boundary_evidence.json`에 검증과 표본을 보존했다.\n\n"
 else:text+='최종 navigation 측정 전이다. preparation-only runtime은 render_startup_preflight에 따로 보존한다.\n\n'
 text+=(ROOT/'REPORT_NOTES.md').read_text()+'\n결과 루트: `'+str(ROOT)+'`\n'
 (ROOT/'REPORT.md').write_text(text)
if __name__=='__main__':main()
