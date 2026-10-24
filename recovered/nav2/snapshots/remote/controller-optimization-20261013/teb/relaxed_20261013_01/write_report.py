import json
import shutil
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parent
WORKSPACE=ROOT.parents[2]

def fmt(v,d=3):
    return 'null' if v is None else f'{v:.{d}f}' if isinstance(v,(int,float)) else str(v)

def table(headers,rows):
    return '| '+' | '.join(headers)+' |\n| '+' | '.join(['---']*len(headers))+' |\n'+''.join('| '+' | '.join(map(str,r))+' |\n' for r in rows)

def write(root=ROOT):
    results=json.loads((root/'results.json').read_text())
    assert len(results)==5 and sorted(r['goal_index'] for r in results)==list(range(5)), 'Need exactly five navigation measurements'
    results.sort(key=lambda r:r['goal_index'])
    relaxed=root.name.startswith('relaxed')
    pilots=json.loads((root/'pilot_results.json').read_text())
    original=[r for r in json.loads((WORKSPACE/'controller-comparison-20261013/remote_results.json').read_text()) if r['candidate']=='teb']
    arrived=sum(r['arrived'] for r in results)
    gt_arrived=sum(r['arrived'] and r['final_gt_tolerance_observed'] for r in results)
    text=f'# TEB {"Gazebo 완화" if relaxed else "보수 공통 제한"} 최적화 — 2026-10-13\n\n'
    text+=f'동결한 설정으로 공통 5개 목표를 각각 한 번 주행했다. Nav2 도착 {arrived}/5, 도착 중 GT 15cm/10도 범위 관측 {gt_arrived}/5다. 합격기준이나 통합점수를 정의하지 않았다. 중단·timeout 행의 오차는 종료 시 남은 거리와 방향 차이며 도착 정확도가 아니다.\n\n'
    if relaxed:
        text+='이 단계는 Gazebo 전용으로 별도 승인된 완화 조건을 적용했다. 기존 vx/vy/wz 박스를 제거하고 r=.0635m, wheelbase=.35m, separation=.6038m, 각 wheel 5rad/s의 역기구학 envelope에 따라 twist 전체를 균일 축소한다. Nav2 → wheel limiter → cmd_vel → Gazebo command guard가 실제 전달 경로다. 최종 wheel limiter는 4.99rad/s의 수치 여유, wheel별 8rad/s² slew, 20Hz sim-time 출력이다. pan/freshness gate, Collision Monitor, voxel guard는 진단을 계속하지만 출력을 ignored/cmd_vel로 돌려 주행을 제한하지 않는다. controller 내부 footprint/feasibility 검사와 실제 robot collision geometry는 유지했다. 물리 로봇용 설정이 아니다.\n\n'
        text+='Progress 시간은 1200sim초, controller failure_tolerance=-1, BT 재시도 10000이다. 기존 240sim초는 소프트 관측 시각이고 hard watchdog은 1200sim/1800wall초다. 이 설정은 예외·정체를 해결한 증거가 아니며 raw compute 예외, 원인 로그, 안전 가드의 counterfactual 진단을 그대로 남겼다. SDF에 Contact 센서를 추가하고 MecanumDrive의 body 축별 ±2 가속 제한을 제거했다. 결합된 wheel envelope와 wheel 가속 제한으로 대체했고 구조/충돌 geometry/물리 dt는 그대로다. 이 actuator 변경은 완화 조건에서 controller 사이를 공정하게 비교할 때 공통 적용해야 한다.\n\n'
    else:
        text+='최종 native 변경은 min_obstacle_dist: 0 → 0.05m 한 가지다. 기존 expanded TEB polygon, costmap footprint/padding, 내부 feasibility, pan/2D/3D 안전 가드, planner, map/world, 속도 박스, 15cm/10도 goal checker를 유지했다. 최종 후보수 5, inner/outer 반복 5/4, dt_ref=.3, 제어 20Hz, RTF 목표1, physics dt=.002, 목표별240sim/360wall초다.\n\n'
        text+='기존 정식 기준선은 Nav2 도착 1/5였고 compute p95 58.9–65.0ms였다. 이번 단일 대표 pilot에서 5cm 간격 설정이 직선 도착과 reset 제거를 보였지만, 다른 실행의 변동과 최종 캠페인을 함께 봐야 한다. 반복 캠페인이나 확률적 성공률 추정은 하지 않았다.\n\n'
    text+='## 원래 정식 기준선\n\n'
    text+=table(['목표','Nav2 상태','sim s','GT XY cm','GT yaw °','compute p95 ms'],
        [[r['goal_name'],r['terminal_status'],fmt(r.get('navigation_sim_s')),fmt(r.get('position_error_m',0)*100),fmt(r.get('yaw_error_deg')),fmt(r.get('controller_compute',{}).get('all_ms',{}).get('p95'))] for r in original])
    text+='\n동일 목표의 아래 동결 결과와 비교한다. 실패 잔여 오차와 성공 도착 오차는 같은 의미가 아니며, 호스트 부하와 AMCL/Gazebo 확률성이 다르므로 일반 성공률을 추정하지 않는다.\n\n'
    text+='## 최종 정확도와 시간\n\n'
    text+=table(['목표','Nav2 상태','sim s','wall s','GT XY cm','GT yaw °','GT 범위 관측','실제 RTF'],
        [[r['goal_name'],r['terminal_status'],fmt(r.get('navigation_sim_s')),fmt(r.get('navigation_wall_s')),fmt(r.get('position_error_m',0)*100),fmt(r.get('yaw_error_deg')),r['final_gt_tolerance_observed'],fmt(r.get('achieved_rtf'))] for r in results])
    text+='\nNav2 상태와 GT 오차는 독립 항목이다. final_nav2_map_xy는 AMCL/TF 좌표이며 GT와 다를 수 있다. stateful SimpleGoalChecker가 XY를 latch할 수 있으므로 Nav2 도착만으로 GT 허용오차 충족을 주장하지 않는다. 시작 map pose [0,0,0]은 기존 시뮬레이션 spawn 좌표계로 변환하며 world 원점과 다르다.\n\n'
    text+='## 연산량과 주기\n\n'
    text+=table(['목표','median ms','p95 ms','p99 ms','예외 수','budget ms','budget 초과','CPU %/core','RSS p95 MiB','Nav2 sim주기 평균 s'],
        [[r['goal_name'],*[fmt(r['controller_compute']['all_ms'].get(k)) for k in ('median','p95','p99')],r['controller_compute'].get('exception_count'),fmt(r['controller_compute']['wall_deadline_ms']),r['controller_compute']['wall_deadline_exceeded'],fmt(r['controller_resources']['one_core_cpu_percent']),fmt(r['controller_resources']['rss_mb'].get('p95')),fmt(r.get('command_sim_interval_s',{}).get('/nav2/cmd_vel',{}).get('mean'))] for r in results])
    text+='\ncomputeVelocityCommands 내부 steady wall 측정이며 정상 반환과 예외를 모두 포함한다. 성공/예외별 통계와 max도 JSON에 있다. 파일 기록은 측정 밖이다. CPU는 controller_server 프로세스의 one-core 100% 기준이고 RSS는 그 프로세스 표본이다. 호스트는 기준선과 같은 Ryzen 5 9600X x86_64; 다른 CPU 호스트와 절대값 순위를 비교하지 않는다. 다른 소유자의 기존 Gazebo 프로세스를 관측했고 종료하지 않았다. 변동하는 병렬 호스트 부하는 측정에 포함된다.\n\n'
    if relaxed:text+='완화 단계의 최종 frequency는 각 result.json의 controller_frequency_hz에 명시했고 compute budget=1000/frequency ms다. TEB dt_ref는 가변 timed-band의 목표 시간 간격이며 고정 model integration dt를 사용하는 controller와 구분한다. 실제 명령 sim주기도 별도 기록했다.\n\n'
    if relaxed:
        text+='최종 native TEB는 single-band (homotopy/multithreading false), inner/outer5/4, dt_ref=.1 및 hysteresis=.02, min_obstacle_dist=.05다. max_vel_x/back/y=.3175m/s, max_vel_theta=.665758rad/s, acc_lim_x/y=.5m/s², acc_lim_theta=.8rad/s²다. 원시 TEB 명령의 조합은 wheel envelope를 넘을 수 있으므로 실제 전달 명령을 별도 제한하며, max_number_classes5는 homotopy 비활성 상태에서 효과가 없다. controller10Hz에 대한 compute wall budget100ms, limiter출력20Hz다.\n\n'
    text+='## GT 부드러움\n\n'
    text+=table(['목표','linear jerk RMS/p95','angular jerk RMS/p95','병진 정지 s','motion stop 수','forward/lateral/yaw 반전'],
        [[r['goal_name'],fmt(r['smoothness'].get('linear_jerk_rms_m_s3'))+'/'+fmt(r['smoothness'].get('linear_jerk_p95_m_s3')),fmt(r['smoothness'].get('angular_jerk_rms_rad_s3'))+'/'+fmt(r['smoothness'].get('angular_jerk_p95_rad_s3')),fmt(r['smoothness'].get('stationary_s')),r['smoothness'].get('motion_stop_count'),'/'.join(str(r['smoothness'].get(k)) for k in ('forward_reversals','lateral_reversals','yaw_reversals'))] for r in results])
    text+='\nGT 속도를 sim time 20Hz에 보간하고 centered 5-point smoothing 뒤 acceleration/jerk를 계산하며 양끝 .25초를 제외했다. stationary는 병진 .01m/s 미만이고 제자리 회전을 포함한다. 장시간 정지한 실패의 낮은 전체 jerk를 좋은 주행으로 해석하지 않는다. raw command stop 수와 GT motion stop 수는 정의가 다르며 둘 다 JSON에 있다.\n\n'
    text+='## 제한·접촉 검증\n\n'
    text+=table(['목표','최종 cmd wheel max','joint wheel max','종료 GT 속도','실측 contact episodes'],
        [[r['goal_name'],fmt(r.get('max_command_wheel_rad_s',{}).get('/cmd_vel')),fmt(r['actual_wheel_feedback_rad_s'].get('max')),r.get('final_actual_velocity'),r.get('actual_contact_count')] for r in results])
    if relaxed:
        text+='\n실측 접촉은 ground_plane 및 robot self-contact를 제외한 collision pair episode 수이며 .25sim초 이상 간격을 새 episode로 센다. 원시 contacts, 깊이, 이름/ID와 floor 접촉 positive control을 보존했다. 접촉 데이터 attribution이 안 되면 null과 이유를 남긴다. Fortress Contact 시스템은 접촉 때만 physics update별로 발행하므로 empty 메시지 부재를 센서 실패로 간주하지 않는다. wheel envelope scaling 표본/축소율/최대 입력·출력 및 2초 wall command watchdog은 wheel_limiter.json과 analysis.json에 있다.\n\n'
    else:
        text+='\n보수 단계에는 physics contact sensor가 없으므로 actual_contact_count=null이다. 실제 충돌 0회로 추론하지 않는다. command_stage_audit는 모든 단계의 속도 박스 초과 수, 마지막 명령과 수신 시각을 포함한다. 마지막 최종 명령과 실제 속도 정지를 별도로 확인했다.\n\n'
    text+='## 기록된 경로의 정적 투영 간격\n\n'
    text+=table(['목표','최소 XY 투영 간격 m','투영 overlap 표본','지원 static collision 수'],[[r['goal_name'],fmt(r.get('footprint_geometry',{}).get('navigation',{}).get('min_projected_clearance_m')),r.get('footprint_geometry',{}).get('navigation',{}).get('projected_overlap_sample_count'),r.get('footprint_geometry',{}).get('obstacle_count')] for r in results])
    text+='\nCAD physical footprint와 static collision의 XY 투영을 기록된 GT 표본에서 비교했다. full-height prism과 mesh convex hull은 서로 다른 높이/틈을 채울 수 있고 inter-sample swept contact를 검증하지 않는다. 이 간격은 물리 Contact 센서 결과와 별도 지표다. geometry_summary.json에 모델·지원 범위·한계·입력 hash를 보존했다.\n\n'
    text+='## 대표 구간 튜닝과 원인\n\n'
    text+=table(['실험','목표','상태','sim s','GT XY cm','compute p95 ms','reset 수'],
        [[r['artifact_group'],r['goal_name'],r['terminal_status'],fmt(r.get('navigation_sim_s')),fmt(r.get('position_error_m',0)*100) if 'position_error_m' in r else 'null',fmt(r.get('controller_compute',{}).get('all_ms',{}).get('p95')),r['infeasible_reset_count']] for r in pilots])
    if not relaxed:
        text+='\n초기 min_obstacle_dist=0는 AddEdgesObstacles의 force-inclusion/cutoff 거리도 0으로 만든다. 양수 거리 장애물이 association에서 제외되어 polygon이 lethal cell을 향해 갈 수 있다. 진단 pilot의 거절은 모두 Trajectory Hits Obstacle였다. min_obstacle_dist=.05만 추가한 pilot은 22.038sim초 도착, GT 2.224cm/6.562°, reset 0, p95 25.696ms였다. 후보수2/inner3/single-band는 연산량을 낮췄지만 명령 방향 변화, TURNING_PAN/WAIT_FRESH_DEPTH, progress 실패가 남아 채택하지 않았다. dt_ref=.4는 lethal footprint 거절을 다시 만들었다.\n\n'
        text+='이후 동결 검증에서 같은 설정의 재현성과 구조적 명령-gate 제약을 평가했다. 원시 command_transitions.json과 pan_gate_reason_duration_sim_s, pan_requested_bearing_step_rad, body_guard_stop_reason_duration_sim_s로 각 원인을 추적한다. 3D guard의 늦은 Command timeout을 최초 controller 실패나 장애물 대기로 오인하지 않는다. 더 강한 공통 안전 계층 완화는 본 단계에 섞지 않고 별도 승인된 완화 단계로 진행한다.\n\n'
    if relaxed:
        text+='\n첫 유효 pilot에서 cmd wheel max5.000이나 joint max5.302rad/s가 측정되었다. Gazebo의 독립 body-axis 가속 제한 대신 convex twist 보간으로 각 wheel 가속을 제한했다. 수정 회전 pilot은32.754sim초, GT2.906cm/1.671°, joint max4.990, contact0; 의자 pilot은23.762sim초, GT7.787cm/1.497°, joint max4.990, contact0이었다. 이는 제어 설정과 공통 actuator 변경을 함께 검증한 단계이며 TEB-only 개선으로 해석하지 않는다. 목표/월드/footprint 및 내부 feasibility는 유지했다.\n\n'
    text+='## 호환성·재현성·한계\n\n'
    text+='Pinned Cleany b3ca816ba420452ee7960837bc16ec74148e07b0, Nav2 Humble1.1.20, Fortress6.18.0, TEB humble-devel630a22e88dc9fd45be726a762edbb5b776bef231이다. latest-TF lookup 및 null converter cleanup 호환 패치를 유지했고 dedicated build/Gazebo에서 검증했다. diagnostics.patch는 거절 이유와 실패 pose/index만 추가한다. 기존 CMake cache의 build type은 비었지만 CMakeLists가 Release를 강제해 기존/새 빌드 모두 -O3 -DNDEBUG였으므로 Release 성능 개선이라고 주장하지 않는다.\n\n'
    infrastructure=json.loads((root/'infrastructure_results.json').read_text())
    text+=f'목표를 발행하기 전 인프라 실패 {len(infrastructure)}개는 infrastructure_results.json/csv에 별도 보존했다. startup retry는 같은 목표의 중복 주행이 아니다. 모든 주행 실패/timeout은 results에 포함했다. 각 목표 한 회이며 Gazebo/AMCL의 같은 random seed를 보장하지 않는다. 단일 pilot의 인과 효과를 확률적 일반 성능 개선으로 확대하지 않는다.\n\n'
    text+='results.json/csv, pilot_results.json/csv, all_runs.json, infrastructure_results.json/csv, final_controller.yaml, frozen_overrides.json, 원시 final/*/bag 및 log/CSV/JSON, REPRO.md를 함께 보존한다. 기존 사용자 worktree/다른 controller 파일을 수정하지 않았고 push하지 않았다. cleanup.json은 이 실험의 정확한 partition/PID만 검사한다.\n'
    (root/'REPORT.md').write_text(text)
    ref=Path(results[0]['run_directory'])
    # Analysis paths are relative to the common optimization/comparison parent.
    run=next(p.parent for p in (root/'final').glob('*/result.json') if json.loads(p.read_text()).get('goal_index')==0)
    source=run/'config/relaxed_parameters.yaml' if relaxed else run/'effective_parameters.yaml'
    final=yaml.safe_load(source.read_text())
    controller=final['controller_server']['ros__parameters']
    f=controller['FollowPath'];f['plugin']=f.pop('wrapped_plugin',f['plugin']);f.pop('benchmark_timing_file',None)
    for name,value in results[0].get('verified_teb_parameters',{}).items():
        f[name.removeprefix('FollowPath.')]=value
    (root/'final_controller.yaml').write_text(yaml.safe_dump({'controller_server':{'ros__parameters':controller}},sort_keys=False))
    shutil.copy2(source,root/'tested_runtime_parameters.yaml')
    return results

if __name__=='__main__':write()
