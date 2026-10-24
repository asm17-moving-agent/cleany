import csv,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
stages={'conservative':ROOT/'resume_20261013_01','gazebo_relaxed':ROOT/'relaxed_20261013_01'}
results={name:json.loads((p/'results.json').read_text()) for name,p in stages.items()}
assert all(len(rows)==5 for rows in results.values())
(ROOT/'results_by_stage.json').write_text(json.dumps(results,indent=2,allow_nan=False))
flat=[]
for name,rows in results.items():
 for r in rows:
  c=r['controller_compute']['all_ms'];w=r['actual_wheel_feedback_rad_s']
  flat.append({'stage':name,'goal':r['goal_name'],'nav2_status':r['terminal_status'],'sim_s':r['navigation_sim_s'],'wall_s':r['navigation_wall_s'],'rtf':r['achieved_rtf'],'gt_xy_m':r['position_error_m'],'gt_yaw_deg':r['yaw_error_deg'],'gt_tolerance_observed':r['final_gt_tolerance_observed'],'compute_median_ms':c['median'],'compute_p95_ms':c['p95'],'compute_p99_ms':c['p99'],'compute_budget_ms':r['controller_compute']['wall_deadline_ms'],'compute_over_budget':r['controller_compute']['wall_deadline_exceeded'],'cpu_one_core_percent':r['controller_resources']['one_core_cpu_percent'],'rss_p95_mib':r['controller_resources']['rss_mb']['p95'],'joint_wheel_max_rad_s':w['max'],'cmd_wheel_max_rad_s':r['max_command_wheel_rad_s']['/cmd_vel'],'actual_contact_episodes':r['actual_contact_count']})
with (ROOT/'results_by_stage.csv').open('w') as f:
 writer=csv.DictWriter(f,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
text='# TEB 최적화 및 검증 완료 — 2026-10-13\n\n'
text+='보수 공통 제한과 별도 승인된 Gazebo 완화 조건을 분리해 최적화하고, 각 단계에서 설정을 동결한 뒤 공통 다섯 목표를 각각 한 번 평가했다. Nav2 도착과 GT 15cm/10° 관측은 별도로 집계한다. 통합 점수나 합격기준은 만들지 않았다.\n\n'
text+='| 조건 | Nav2 도착 | 도착 중 GT 범위 관측 | compute p95 범위 ms | joint wheel 최대 rad/s | 물리 접촉 episodes |\n| --- | --- | --- | --- | --- | --- |\n'
for name,rows in results.items():
 count=sum(r['arrived'] for r in rows);gt=sum(r['arrived'] and r['final_gt_tolerance_observed'] for r in rows)
 p95=[r['controller_compute']['all_ms']['p95'] for r in rows];wheel=max(r['actual_wheel_feedback_rad_s']['max'] for r in rows)
 contact='null (센서 없음)' if name=='conservative' else str(sum(r['actual_contact_count'] for r in rows))
 text+=f'| {name} | {count}/5 | {gt}/5 | {min(p95):.2f}–{max(p95):.2f} | {wheel:.3f} | {contact} |\n'
text+='\n보수 단계는 min_obstacle_dist=0.05m만 최종 native 변경으로 채택했다. 원래 기준선의 Nav2 도착은1/5였다. 보수 단계에서 pan 방향 전환·깊이 freshness 대기와 progress 실패가 남았다. 완화 단계는 이 별도 전달 가드를 우회하고 wheel envelope·가속 제한, watchdog, TEB search/time discretization 및 controller 주기를 함께 바꿨다. 따라서 두 단계 차이를 TEB-only 개선으로 해석하거나 다른 controller와 그대로 순위 비교하지 않는다.\n\n'
text+='완화 단계의 실제 경로는 Nav2 → uniform wheel-envelope scaling → wheel별 slew를 제한하는 convex twist 보간 → cmd_vel → Gazebo guard다. r=.0635m, wheelbase=.35m, separation=.6038m, 물리 wheel cap5rad/s를 유지한다. 수치 여유를 위해 출력 cap4.99, wheel 가속8rad/s², 출력20Hz를 채택했다. 기존 Gazebo body축 가속 제한을 사용한 초기 pilot에서 joint5.302rad/s 초과가 관측됐다. 독립 축 보간이 결합 envelope를 벗어날 수 있어 이를 결합 limiter로 대체했다. 이 actuator 변경은 완화 조건의 공정한 controller 비교에서 공통 적용해야 한다. 실제 collision geometry, padded footprint, 내부 feasibility, map/world와 목표는 유지했다. 물리 로봇 적용 설정이 아니다.\n\n'
text+='기존240sim초는 완화 단계의 소프트 표식이며 hard watchdog은1200sim/1800wall초다. goal checker는 그대로15cm/10° stateful이다. RTF 목표1, 실제 주기·RTF·wall compute budget·CPU/RSS·GT jerk·정지/반전·오류/예외·접촉 및 wheel 명령/joint 값은 아래 원시 결과에 남겼다. 보수 단계의 접촉은 계측되지 않았으므로0으로 간주하지 않는다. 완화 단계 접촉은 full rosbag2 물리 collision-pair episode이며 floor positive control을 검증했다.\n\n'
text+='| 목표 | 보수 Nav2 / sim s / GT cm,° | 완화 Nav2 / sim s / GT cm,° |\n| --- | --- | --- |\n'
for a,b in zip(sorted(results['conservative'],key=lambda r:r['goal_index']),sorted(results['gazebo_relaxed'],key=lambda r:r['goal_index'])):
 def cell(r):return f"{r['terminal_status']} / {r['navigation_sim_s']:.3f} / {r['position_error_m']*100:.2f}, {r['yaw_error_deg']:.2f}"
 text+=f"| {a['goal_name']} | {cell(a)} | {cell(b)} |\n"
text+='\n실패/timeout의 GT 오차는 종료 시 잔여 거리·방향이며 도착 정확도가 아니다. 각 목표 한 회이므로 일반 성공률을 추정하지 않는다. 기존/새 TEB는 모두-O3 -DNDEBUG였고 Release 빌드 성능 개선이라고 주장하지 않는다. 다른 CPU 호스트와 절대 compute/CPU 순위를 비교하지 않는다.\n\n'
for name,p in stages.items():
 text+=f'- [{name} 상세 REPORT]({p}/REPORT.md): 원인, 대표 pilot, 최종 정확도·시간, median/p95/p99, budget/예외, CPU/RSS, GT jerk, 접촉·제한 검증.\n'
 text+=f'- [{name} JSON]({p}/results.json), [CSV]({p}/results.csv), [최종 native YAML]({p}/final_controller.yaml), [재현 절차]({p}/REPRO.md). 원시 final 목표별 bag/log/CSV/JSON, 호환·진단·runner patch, source/build archive 및 환경 메타데이터를 함께 보존했다.\n'
text+=f'\n[단계별 통합 JSON]({ROOT}/results_by_stage.json), [CSV]({ROOT}/results_by_stage.csv), [완화 artifact 검증]({stages["gazebo_relaxed"]}/artifact_checks.json).\n\n'
text+=f'![최종 GT 경로]({ROOT}/final_trajectories.png)\n\n'
text+='Full access에서 기존 소유 시뮬레이션을 확인하고 진행했다. 소유 partition/PID만 정리했고 다른 작업의 프로세스·파일과 기존 dirty checkout을 보존했다. dedicated Cleany source는 pinned b3ca816ba420452ee7960837bc16ec74148e07b0의 clean 상태다. push하지 않았다. cleanup.json에 각 단계의 소유 프로세스가 남지 않았음을 기록했다.\n'
(ROOT/'REPORT.md').write_text(text)
print(text[:1100])
