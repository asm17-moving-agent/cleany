import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import mean

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent
records = []
sources = []

def add(path, data, stage=None, controller=None, pointer='', source_indices=None):
    sources.append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    for i, x in enumerate(data):
        s = stage or x.get('stage')
        s = {'baseline_historical':'baseline','conservative_final':'conservative','relaxed_final':'relaxed','gazebo_relaxed':'relaxed','relaxed_gazebo_only':'relaxed'}.get(s,s)
        smooth = x.get('smoothness', {})
        recomputed = x.get('smoothness_recomputed', {})
        comp = x.get('controller_compute', {})
        times = x.get('compute_ms', comp.get('all_ms', {}))
        resources = x.get('controller_resources', {})
        dwb_resources = x.get('controller_process_resources', [])
        commands = x.get('max_command_wheel_rad_s', {})
        collision = x.get('collision_measurement', {})
        pos = x.get('position_error_cm', x.get('position_error_m', math.nan)*100)
        arrived = x.get('action_status') == 4
        cname = controller or x['candidate']
        def metric(flat, nested, recompute=None):
            return x.get(flat, recomputed.get(recompute, smooth.get(nested))) if recompute else x.get(flat, smooth.get(nested))
        r = dict(controller=cname,stage=s,goal=x['goal_name'],goal_index=x['goal_index'],host=x.get('host'),
            action_status=x.get('action_status'),nav2_arrived=arrived,
            gt_xy_cm=pos,gt_yaw_deg=x.get('yaw_error_deg'),
            gt_within_reference_at_arrival=arrived and pos<=15 and x.get('yaw_error_deg',math.inf)<=10,
            error_meaning='arrival error' if arrived else 'remaining error at abort/timeout',
            sim_s=x.get('sim_s',x.get('navigation_sim_s')),wall_s=x.get('wall_s',x.get('navigation_wall_s')),
            rtf=x.get('rtf',x.get('achieved_rtf')),
            compute_median_ms=x.get('compute_median_ms',times.get('median')),
            compute_p95_ms=x.get('compute_p95_ms',times.get('p95')),
            compute_p99_ms=x.get('compute_p99_ms',times.get('p99')),
            compute_exceptions=x.get('compute_exception_calls',times.get('exception_count',comp.get('exception_count',comp.get('exception_ms',{}).get('count')))),
            compute_over_50ms=x.get('compute_over_50ms',times.get('over_50ms',comp.get('reference_over_50ms_count',comp.get('wall_deadline_exceeded')))),
            controller_frequency_hz=x.get('controller_frequency_hz',10 if cname=='teb' and s=='relaxed' else 20),
            final_output_frequency_hz=20,
            cpu_one_core_percent=x.get('controller_cpu_one_core_pct',resources.get('one_core_cpu_percent',dwb_resources[0].get('cpu_percent_one_core') if dwb_resources else None)),
            rss_p95_mib=x.get('controller_rss_p95_mb',resources.get('rss_mb',{}).get('p95')),
            rss_max_mib=dwb_resources[0].get('RSS_max_MiB') if dwb_resources else resources.get('rss_mb',{}).get('max'),
            linear_jerk_rms=metric('linear_jerk_rms_m_s3','linear_jerk_rms_m_s3','linear_jerk_rms'),
            linear_jerk_p95=metric('linear_jerk_p95_m_s3','linear_jerk_p95_m_s3','linear_jerk_p95'),
            angular_jerk_rms=metric('angular_jerk_rms_rad_s3','angular_jerk_rms_rad_s3','angular_jerk_rms'),
            angular_jerk_p95=metric('angular_jerk_p95_rad_s3','angular_jerk_p95_rad_s3','angular_jerk_p95'),
            stationary_sim_s=metric('stationary_s','stationary_s','stationary_sim_s'),
            stationary_definition='translation<.01m/s AND yaw<.02rad/s' if cname=='mppi' and s!='baseline' else 'translation<.01m/s; rotation included',
            stops=recomputed.get('motion_stop_count',smooth.get('stop_count',smooth.get('motion_stop_count'))),
            motion_starts=metric('motion_start_count','motion_start_count'),
            yaw_reversals=metric('yaw_reversals','yaw_reversals','yaw_reversal_count'),
            lateral_reversals=metric('lateral_reversals','lateral_reversals','lateral_reversal_count'),
            final_command_wheel_max_rad_s=x.get('max_cmd_vel_wheel_rad_s',commands.get('/cmd_vel')),
            raw_controller_wheel_max_rad_s=x.get('max_nav2_cmd_vel_wheel_rad_s',commands.get('/nav2/cmd_vel')),
            measured_joint_wheel_max_rad_s=x.get('max_measured_wheel_rad_s',x.get('measured_wheel_max_rad_s',x.get('actual_wheel_feedback_rad_s',{}).get('max'))),
            environment_contact_episodes=x.get('environment_contact_episodes',collision.get('environment_contact_episodes',x.get('actual_contact_count'))),
            source_json=str(path),source_pointer=f'{pointer}/{source_indices[i] if source_indices is not None else i}',raw_run_directory=x.get('run_directory',x.get('result_path')),
            external_guards_bypassed=s=='relaxed',physical_validation=False,
            minimum_true_clearance_m=None,minimum_true_clearance_null_reason='No common continuous true clearance measurement; projected static geometry is not physical clearance',
            penetration_depth_m=None,penetration_depth_null_reason='No common measured contact penetration depth',
            null_note='Missing source metrics remain null. Historical per-cycle compute often unmeasured. DWB RSS is max, not p95. Conservative contact sensing absent.')
        if cname=='mppi' and s!='baseline':
            r['stationary_sim_s']=x.get('full_stationary_s')
        records.append(r)

def read(path): return json.loads(path.read_text())
baseline=ROOT.parent/'controller-comparison-20261013/comparison.json'
baseline_rows=read(baseline)
baseline_indices=[i for i,x in enumerate(baseline_rows) if x['candidate'] in ['mppi','teb']]
add(baseline,[baseline_rows[i] for i in baseline_indices],stage='baseline',source_indices=baseline_indices)
dwb=ROOT/'dwb/comparison_metrics.json'; add(dwb,read(dwb))
mppi=ROOT/'mppi/sol_resume_20261013/results.json'; add(mppi,read(mppi))
teb=OUT/'source_snapshots/teb/results_by_stage.json'
for s,rows in read(teb).items():add(teb,rows,stage=s,pointer='/'+s)
rpp=OUT/'source_snapshots/rpp/results.json'
for s,rows in read(rpp)['phases'].items():add(rpp,rows,stage=s,pointer='/phases/'+s)

assert len(records)==60
assert len({(r['controller'],r['stage'],r['goal']) for r in records})==60
assert all(r['sim_s'] is not None and math.isfinite(r['gt_xy_cm']) for r in records)
relaxed=[r for r in records if r['stage']=='relaxed']
assert len(relaxed)==20 and all(r['nav2_arrived'] for r in relaxed)
assert all(r['final_command_wheel_max_rad_s']<=5+1e-7 and r['measured_joint_wheel_max_rad_s']<=5+1e-7 for r in relaxed)
assert all(r['environment_contact_episodes']==0 for r in relaxed)
def savecsv(path,rows):
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
savecsv(OUT/'comparison.csv',records)
summaries=[]
for c in ['dwb','mppi','teb','rpp']:
    for s in ['baseline','conservative','relaxed']:
        rows=[r for r in records if r['controller']==c and r['stage']==s]
        ok=[r for r in rows if r['nav2_arrived']]
        def avg(k, group=rows):
            vals=[r[k] for r in group if r[k] is not None];return mean(vals) if vals else None
        def span(k):
            vals=[r[k] for r in rows if r[k] is not None];return [min(vals),max(vals)] if vals else None
        summaries.append(dict(controller=c,stage=s,nav2_arrived=len(ok),goals=5,gt_within_reference_at_arrival=sum(r['gt_within_reference_at_arrival'] for r in rows),
            total_recorded_sim_s=sum(r['sim_s'] for r in rows),total_time_comparable_all_arrived=len(ok)==5,
            mean_arrival_gt_xy_cm=avg('gt_xy_cm',ok),mean_arrival_gt_yaw_deg=avg('gt_yaw_deg',ok),
            mean_goal_linear_jerk_rms=avg('linear_jerk_rms'),mean_goal_angular_jerk_rms=avg('angular_jerk_rms'),
            compute_p95_ms_range=span('compute_p95_ms'),cpu_percent_range=span('cpu_one_core_percent'),rtf_range=span('rtf'),
            max_final_command_wheel_rad_s=max(r['final_command_wheel_max_rad_s'] for r in rows),
            max_joint_wheel_rad_s=max(r['measured_joint_wheel_max_rad_s'] for r in rows),
            contact_episodes=sum(r['environment_contact_episodes'] for r in rows) if all(r['environment_contact_episodes'] is not None for r in rows) else None))
(OUT/'comparison.json').write_text(json.dumps({'schema_version':1,'records':records,'summaries':summaries,'sources':sources,'aggregation':'Each goal weighted equally for jerk means; total sim time only comparable when all five arrived. No composite score or statistical success probability.'},indent=2,allow_nan=False))
savecsv(OUT/'summary.csv',summaries)

lines=['# Nav2 개선·완화 재실험 비교 — 2026-10-13 KST','',
'공통 목표 5개를 단계별로 각각 한 번 실행했다. 완화 최종은 네 컨트롤러 모두 Nav2 도착 5/5였고, 실제 GT 15cm/10° 관측은 DWB·MPPI·RPP 각각4/5, TEB5/5다. 이는 관측 건수이며 합격 기준·통계적 성공률·통합 점수는 없다.','',
'## 단계별 관측','',
'| 컨트롤러 | 원래 → 기존 제한 개선 → 완화 Nav2 도착 | 완화 GT 범위 관측 | 완화 5목표 합계 sim초 | 완화 평균 GT cm / ° |',
'|---|---|---|---|---|']
for c in ['dwb','mppi','teb','rpp']:
    ss=[x for x in summaries if x['controller']==c]; r=ss[-1]
    lines.append(f"| {c.upper()} | {' → '.join(str(x['nav2_arrived'])+'/5' for x in ss)} | {r['gt_within_reference_at_arrival']}/5 | {r['total_recorded_sim_s']:.2f} | {r['mean_arrival_gt_xy_cm']:.2f} / {r['mean_arrival_gt_yaw_deg']:.2f} |")
lines += ['', '원래·보수 단계의 실패/timeout 종료 오차는 남은 거리이며 도착 정확도가 아니다. 실패를 포함한 총 소요시간으로 빠르다고 판단하지 않는다. DWB 역사적 baseline에는 의도된 속도 박스 위반이 발견되어 동일 조건 A/B로 간주하지 않는다. MPPI 최종 보수는 distant abort로 4/5이며, 이전 진행 메시지의 모두 도착 진술은 최종 기록으로 대체한다.', '',
'## 완화 목표별 정확도·시간','', '| 컨트롤러 | 목표 | sim초 | GT cm | GT ° | GT 15cm/10° 관측 |','|---|---|---|---|---|---|']
for r in relaxed: lines.append(f"| {r['controller'].upper()} | {r['goal']} | {r['sim_s']:.3f} | {r['gt_xy_cm']:.3f} | {r['gt_yaw_deg']:.3f} | {'범위 내' if r['gt_within_reference_at_arrival'] else '범위 밖'} |")
lines += ['', '의자 사이 목표 GT XY는 DWB16.36cm, MPPI16.71cm, RPP16.02cm로 남았다. Nav2는 추정 pose를 사용하며 마지막 GT/TF는 비동기 관측이다. 이 차이를 tolerance 확대나 재실행으로 숨기지 않았다. TEB의 이 목표 GT XY는6.62cm다.', '',
'## 연산량·부드러움','', '| 컨트롤러 / 호스트 | 주기 | 목표별 compute p95 범위 ms | 평균 선형 jerk RMS m/s³ | 평균 각 jerk RMS rad/s³ | joint 최대 rad/s |', '|---|---|---|---|---|---|']
for c in ['dwb','mppi','teb','rpp']:
    s=next(x for x in summaries if x['controller']==c and x['stage']=='relaxed'); span=s['compute_p95_ms_range']
    lines.append(f"| {c.upper()} / {'ARM64' if c in ['dwb','mppi'] else 'x86'} | {10 if c=='teb' else 20}Hz | {span[0]:.3f}–{span[1]:.3f} | {s['mean_goal_linear_jerk_rms']:.3f} | {s['mean_goal_angular_jerk_rms']:.3f} | {s['max_joint_wheel_rad_s']:.4f} |")
lines += ['', 'compute는 controller 내부 steady wall 시간이다. DWB·MPPI는 같은 ARM64, TEB·RPP는 같은 x86에서 측정했다. 서로 다른 호스트 절대 compute/CPU 순위를 만들지 않는다. RPP의 별도 Python wheel limiter 비용은 controller compute에 포함되지 않는다. CPU/RSS·median/p99·주기·RTF·예외·50ms 초과는 comparison.csv/JSON에 목표별로 보존했다. DWB RSS max와 나머지 RSS p95를 같은 지표로 합치지 않았다.', '',
'GT 속도20Hz sim-time 보간, centered5point smoothing, 양끝.25초 trim 후 jerk를 계산했다. 표는 각 목표 RMS의 동일 가중 산술 평균이며 pooled RMS가 아니다. 전체20목표가 도착한 완화 단계끼리만 이 요약을 제시했다. MPPI는 선형·각 jerk 평균이 낮게 관측됐으며 RPP·TEB는 방향 변화가 더 많았다. 속도·wheel slew·padding·주기가 달라 controller 고유 특성의 단독 증거는 아니다.', '',
'정지 정의: DWB/TEB/RPP는 병진속도<.01m/s라 제자리 회전을 포함하지만 MPPI 새 결과는 병진<.01 AND 각속도<.02다. 각 축 반전·motion starts/stops는 결과에 보존하되 정지 횟수는 정의가 달라 직접 순위화하지 않는다. 오래 정지한 실패의 낮은 jerk를 개선으로 해석하지 않는다.', '',
'## 설정 차이와 해석','',
'- 공통: 휠 반경.0635m, wheelbase.35m, separation.6038m, 실제 footprint/물리 collision geometry, 지도·5목표·StraightFirstPlanner, goal checker15cm/10° 유지. 완화는 외부 pan/freshness/CollisionMonitor/voxel veto를 shadow 출력으로 분리하고 최종 wheel envelope를 적용했다. 내부 collision/feasibility 검사는 유지했다. 240sim초 소프트 표식, hard1200sim/1800wall초다. 실제 wheel command와 joint는 최종20건 모두5rad/s 이내다. 원시 TEB/RPP 명령은 limiter 앞에서5를 넘을 수 있어 최종 출력과 구분했다.',
'- DWB: rollout 안에서 coupled cap4.99 적용, 5×5×7 샘플과 목표 방향 탐색, 목표 회전 critic lookahead1초. full collision rollout2초 유지. 20Hz, native acceleration tuning, 추가 padding.02/inflation.8 유지. 별도 wheel slew는 쓰지 않았다.',
'- MPPI: 도착 XY 탐색 억제와 필터 후 제한 재적용, path orientation critic 조정 및 gamma tuning. optimized/sampled sequence와 최종 출력에 coupled cap 적용. 20Hz, 기존 footprint/costmap 유지. 별도8rad/s² wheel slew는 쓰지 않았다.',
'- TEB: min_obstacle_dist.05, homotopy OFF/single band, dt_ref.1, controller10Hz/출력20Hz. cap4.99 및 wheel slew8rad/s², actuator의 독립 축 제한 대신 coupled slew 적용. 확장 footprint/padding 유지.',
'- RPP: 곡률 보존 vx/wz 축소, lookahead 하한.18m, vx.30/wz.35, native collision TTC1초. cap4.80 및 wheel slew8rad/s², 추가 여유.08m와 costmap padding.02 제거. 실제 CAD footprint와 inflation.8 유지. controller/출력20Hz.',
'',
'현재 결과는 각 controller에 맞춘 전체 프로파일 비교다. 속도 상승·가드 우회·wheel slew·footprint 여유·주기가 동시에 바뀌어 순수 controller 효과를 분리할 수 없다. 사용자 요청한5목표 각1회 재실험을 완료했으므로 추가 반복은 수행하지 않았다. 동일 조건의 엄격한 controller 비교가 필요하면 actuator/slew와 추가 footprint 여유를 통일한 별도 단계가 필요하다.', '',
'관측상 DWB는5목표 합계가 가장 짧았고, MPPI는 각속도 jerk가 낮았다. TEB는 이번20건 중 유일하게 모든 목표의 GT15cm/10° 범위에 들어왔다. RPP는 같은 x86의 TEB보다 native compute 비용이 매우 작았지만 각속도 jerk와 방향 반전이 컸다. 모든 판단은 이번 단일 실행과 각 설정에 한정된다.', '',
'## 접촉·미측정·운영 기록','',
'완화 최종20건은 기록된 환경 접촉0 episode다. floor heartbeat와 collision 이름, DWB의 별도 body positive control로 센서 경로를 확인했다. contact sensor 주기/episode 정의는 다르며 zero 관측은 연속적인 무충돌 보장이나 물리 검증이 아니다. 보수 단계 접촉은 미계측으로 null이다. 연속 최소 실제 clearance와 penetration depth는 공통 미측정으로 null이며 별도 정적 XY 투영 간격을 실제 접촉·clearance로 대체하지 않았다.', '',
'TEB 초기 pilot joint5.302rad/s, RPP 무-slew pilot5.493rad/s 초과 기록은 진단 자료이며 최종 결과에서 제외됐다. 최종 cap/slew 적용 결과와 섞지 않았다. RPP 정리 중 기존 legacy launch와 그 하위 별도 friction Gazebo를 함께 종료한 실수가 보고됐다. 해당 당시 상태/경위는 RPP REPORT와 legacy_owned_processes.json에 보존됐다. 전체 물리 로봇 적용 및 git push는 없었다.', '',
'## 자료·재현','',
'- comparison.json/CSV:60행(4controllers×3stages×5goals), null 이유와 원본 경로, source snapshots 및 SHA256. summary.csv:12단계 요약.',
'- build_comparison.py: 비교 생성 및20개 완화 결과의 상태/command·joint 상한/접촉 검증. source_snapshots: 읽어온 원격 TEB/RPP 보고서와 결과. 원시 bag은 각 원본 경로에 그대로 보존했다.',
f'- 로컬 DWB 보고서: {ROOT}/dwb/REPORT.md',
f'- 로컬 MPPI 보고서: {ROOT}/mppi/sol_resume_20261013/REPORT.md',
'- 원격 TEB 보고서: /home/donglee/workspace/cleany/controller-optimization-20261013/teb/REPORT.md',
'- 원격 RPP 보고서: /home/donglee/workspace/cleany/controller-optimization-20261013/rpp/REPORT.md',
'- 각 보고서에 최종 YAML/patch/reproduce/raw bag/log/runtime parameter checks가 연결되어 있다.',
'', '재생성: `python3 build_comparison.py`. 이 스크립트는 보존된 스냅샷을 읽으며 실험을 재실행하지 않는다.']
(OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(summaries,indent=2))
