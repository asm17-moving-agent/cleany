#!/usr/bin/env python3
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def fmt(x,d=3):return 'null' if x is None else f'{x:.{d}f}' if isinstance(x,(float,int)) else str(x)
def table(headers,rows):
    def cell(value):return ('null' if value is None else str(value)).replace('|', '\\|')
    return '\n'.join(['| '+' | '.join(map(cell,headers))+' |','| '+' | '.join(['---']*len(headers))+' |',*['| '+' | '.join(map(cell,row))+' |' for row in rows]])
def main():
    data=json.loads((ROOT/'results.json').read_text());phases=data['phases'];con=phases['conservative'];rel=phases['relaxed'];base=phases['baseline']
    assert len(con)==len(rel)==len(base)==5
    labels={'baseline':'원래','conservative':'기존 기준 개선','relaxed':'완화 후'}
    arrivals={key:sum(r['action_status']==4 for r in rows) for key,rows in phases.items()}
    position=[];compute=[];smooth=[];bounds=[]
    for key,rows in phases.items():
        for r in rows:
            position.append([labels[key],r['goal_name'],r['terminal_status'],fmt(r['navigation_sim_s']),fmt(r['position_error_m']*100,2),fmt(r['yaw_error_deg'],2),fmt(r.get('nav2_localized_position_error_m')*100 if r.get('nav2_localized_position_error_m') is not None else None,2),fmt(r.get('nav2_localized_yaw_error_deg'),2)])
            c=r['controller_compute'];s=c['all_ms'];a=r['controller_resources'];j=r['smoothness'];stage=r['command_stages']['/cmd_vel']
            compute.append([labels[key],r['goal_name'],fmt(s['median']),fmt(s['p95']),fmt(s['p99']),r['controller_exception_count'],c['wall_deadline_exceeded'],fmt(a['one_core_cpu_percent'],1),fmt(a['rss_mb'].get('p95'),1),fmt(r['achieved_rtf'])])
            if key!='baseline':
                smooth.append([labels[key],r['goal_name'],fmt(j['linear_jerk_rms_m_s3']),fmt(j['linear_jerk_p95_m_s3']),fmt(j['angular_jerk_rms_rad_s3']),fmt(j['angular_jerk_p95_rad_s3']),fmt(j['stationary_s'],1),j['stop_count'],j['yaw_reversals'],j['lateral_reversals']])
                bounds.append([labels[key],r['goal_name'],','.join(fmt(x) for x in stage['max_abs_xyz']),fmt(r['max_command_wheel_rad_s']['/cmd_vel']),fmt(r['actual_wheel_feedback_rad_s']['max']),r.get('actual_contact_count'),r['final_stop_verified']])
    text=f'''# RPP Gazebo 최적화 — 기존 기준과 완화 프로파일

두 단계의 최종 공통 5개 목표를 각각 한 번씩 완료했다. 원래 RPP는 {arrivals['baseline']}/5 도착, 기존 기준 개선은 {arrivals['conservative']}/5 도착, 완화 후는 {arrivals['relaxed']}/5 도착했다. 합격 기준이나 통합 점수는 만들지 않았다. Nav2 도착 판정과 GT 오차는 별도 관측이며, 각 한 회이므로 통계적 성능 보장은 아니다.

## 종료 상태·정확도·시간

도착 행의 GT 오차는 종료 후 2 sim초 정지 관측에서 얻은 오차다. 중단·취소 행은 **남은 목표 거리/방향**이고 도착 정확도가 아니다. Nav2 열은 최종 `map->base_link` 추정 오차다. 원래 기록의 최종 Nav2 추정 오차는 미계측으로 null이다.

{table(['단계','목표','Nav2 종료','주행 sim초','GT XY cm','GT yaw °','Nav2 XY cm','Nav2 yaw °'],position)}

기존 기준에서는 straight/turn_arrival/far_heading의 시간과 오차가 조금 변했지만, 성공 수는 그대로였다. chair_gap의 남은 거리만 원래 138.3cm에서 최종 {con[2]['position_error_m']*100:.1f}cm로 줄었으며 yaw 문제와 240초 정체는 남았다. distant의 남은 거리는 더 커졌으므로 일괄 개선이라고 하지 않는다. 완화 단계의 차이는 속도 제한, 추가 여유, 전달 가드 및 slew가 함께 달라진 결과다.

## 기존 기준에서 진단·선정한 변경

- 원래 소스 HEAD `b3ca816ba420452ee7960837bc16ec74148e07b0`, 별도 `../rpp-source`/`../rpp-build-v2`를 재사용했다. 사용자 checkout과 다른 controller 파일은 수정하지 않았다.
- 첫 정지는 RPP 충돌 예외였다. 원래 첫 예외는 sim39.142초, `/nav2/cmd_vel`은 지속적인 0으로 바뀌었다. 이후 Collision Monitor가 `stop_pub_timeout=2`에 따라 0 발행을 멈추고 뒤쪽 guard가 Command timeout을 표시했다. 원래 206.858sim초 timeout을 장애물 대기나 센서 지연으로 분류하지 않는다. 단계별 원본 timestamp와 `command_event_timeline.csv`를 보존했다.
- native RPP tracking은 `w=v*curvature`이며 공통 guard가 yaw만 .14로 자르면 반경이 달라진다. `CurvatureLimitedRPP`는 **native footprint/예측 검사를 먼저 실행**하고 반환된 vx/wz를 같은 비율로 줄인다. 새 configure-time `tracking_yaw_limit=.14`는 변경 요청을 거절한다. native 선형 가속도 제한이나 smoother는 추가하지 않았다.
- lookahead 하한 .30→.20→.18m를 시험해 최종 .18m를 선정했다. .12m 후보는 carrot 거리가 goal XY tolerance .15보다 작아 시작점에서 목표 회전을 반복했으며 병진 이동 없이 87.028sim초 후 중단됐다. 회전 속도 .10 후보는 충돌/재계획을 반복해 대표 구간에서 조기 취소했고 최종에는 쓰지 않았다. 짧은 TTC .35도 원래 정체를 해결하지 못해 최종 TTC는 1.0으로 유지했다.
- 기록 costmap/TF에서 현재 tangent 방향 footprint 및 목표 방향까지 중간 회전이 lethal 셀에 걸리는 반면, 같은 XY의 목표 차체 방향은 비충돌인 샘플을 얻었다. RPP의 전진 방향 추종과 고정 차체 방향을 사용하는 holonomic planner 경로의 구조적 차이가 남는다. `costmap_diagnosis.json/.npz`는 비동기 공개 grid 재구성으로, native 판정 자체를 대체하지 않는다.
- `07_aux_zero_refresh`는 동일 short-TTC 후보에 **공통 계층** `stop_pub_timeout=300`만 추가한 보조 실험이다. 지속적인 0 전달이 유지되고 navigation 구간 Command timeout 진단은 사라졌지만 native 충돌 예외 35개와 중단은 남았다. 이 변경은 기존 기준 최종 캠페인에 넣지 않았다. 공통 비교에서 적용하려면 모든 controller에 같은 변경이 필요하다.

기존 기준 최종은 box |vx|≤.15, |vy|≤.10, |wz|≤.14, wheel≤5를 유지했다. 추가 footprint .08m와 costmap padding .02m, 안전 검사와 15cm/10° goal checker도 유지했다. 최종 5개 모두 box, 계산 wheel, 관측 joint wheel 제한과 마지막 명령/GT 속도 0을 확인했다.

## 승인된 Gazebo 전용 완화 프로파일

- per-axis box를 제거했다. r=.0635m, k=(.35+.6038)/2=.4769m로 `[(vx-vy-k*wz),(vx+vy+k*wz),(vx+vy-k*wz),(vx-vy+k*wz)]/r`를 계산하고 전체 twist를 동일 배율로 줄인다. 이론적 pure straight wheel5 한계는 .3175m/s다. native desired vx는 .30m/s, 회전 명령 .35rad/s로 선정했다.
- 첫 4.90rad/s 무-slew 후보는 chair_gap 도착에도 실제 joint가 **5.493rad/s**까지 순간 초과했다. sim18.008초 부근의 vx/wz 급변과 wheel 피드백을 보존했다. 해당 후보와 취소한 무-slew distant는 **진단 자료**이며 정식 최종 결과에 섞지 않는다.
- 최종 command wheel 여유는 4.80rad/s, wheel acceleration/deceleration은 8rad/s²다. 목표를 먼저 결합 제한한 후 이전 twist와 목표 사이의 변경을 동일 배율로 줄여 모든 wheel의 변화율을 제한한다. cap와 slew를 함께 바꾼 후보이므로 각 변수의 단독 기여는 분리하지 않았다. slew 중 경로는 native가 검사한 단일 arc와 달라질 수 있어 실제 접촉 계측을 같이 사용했다. 명령 timeout .5초나 invalid 값은 즉시 0으로 보낸다.
- 실제 CAD physical footprint는 유지하고 추가 .08m 여유 및 .02m padding만 제거했다. inflation radius .8m, 동일 지도·센서·StraightFirstPlanner와 native RPP collision detection/TTC1.0은 유지했다. planner algorithm이나 model geometry는 바꾸지 않았다.
- `/nav2/cmd_vel -> wheel_limit -> /cmd_vel`이 최종 권한을 갖는다. 기존 pan/Collision Monitor/voxel freshness·장애물·box 가드는 센서·costmap 처리를 위해 계속 실행하지만 출력은 `/legacy_guard/cmd_vel`로 보내 **주행 veto를 우회**한다. Nav2 내부 검사와 별도 .5초 command watchdog은 남는다. 이 프로파일은 물리 로봇에 적용하지 않았다.
- progress threshold를 .001m/.001rad, allowance1200sim초, controller failure tolerance1200으로 완화했다. 240sim초는 soft schedule, 목표당 hard1200sim초/wall1800초다. goal checker .15m/10°는 유지했다. 최종 frequency는 Nav220Hz 및 final command20Hz sim timer이며 RPP에는 model_dt parameter가 없어 null/not applicable이다.
- 6개 contact sensors가 31개 기존 collision 요소를 모두 관측한다. contact plugin/sensors만 추가하고, 이들을 제거한 XML이 추가 전 scene과 같은 것을 assert했다. 물리 모델·충돌 형상·friction·joint는 그대로다. Fortress Contact 시스템은 SDF의 update_rate50을 실제로 따르지 않고 contact가 있을 때 physics step마다 발행한다. 관측 간격은 JSON에 남겼다.
- 정식 contact 수는 raw bag의 physics contacts를 sim header timestamp로 필터링해 distinct collision pair별 gap≤.25초를 한 episode로 합친 것이다. ground_plane wheel 접촉은 제외하고 양성 대조로 사용한다. GT 기반 보수적 정적 footprint 투영도 별도로 저장한다. 이 투영은 높이/mesh convex hull 때문에 실제 접촉을 과장할 수 있으며 physics contact와 다른 지표다.

## 계산 시간과 자원

inner `computeVelocityCommands` steady wall 시간은 성공·예외 모두 포함한다. 파일 flush는 측정 밖이며 scheduler·native 로그/시각화는 포함한다. Nav2 전체 루프 시간과 limiter 시간은 별도다. 20Hz wall 예산은 50ms다. CPU는 controller_server process의 한 코어100% 기준이고 RSS는 p95 MB다.

{table(['단계','목표','median ms','p95 ms','p99 ms','예외','>50ms','CPU %','RSS p95 MB','실제 RTF'],compute)}

모두 같은 Ryzen5 9600X / `142spp-endeavour` 호스트다. 원래는 TEB와 병렬이었고 새 실행의 배경 부하도 달라 절대 수치를 알고리즘 일반 순위로 해석하지 않는다. 다른 CPU 호스트와 비교하지 않았다. RTF 목표1, physics dt .002초를 유지했고 실제 명령/compute sim·wall 주기와 큰 gap을 JSON에 보존했다. 최종 실제RTF는 기존 .964–.975, 완화 .914–.982였다. Nav2 wall timer 때문에 RTF 차이는 sim-time 호출 밀도에도 영향을 주며, 계측/배경 부하 차이를 포함한 비교 제한으로 남긴다. relaxed limiter의 update 시간과 CPU/RSS는 `limiter_compute_us`/`limiter_resources`로 별도 기록했다.

## 부드러움과 바퀴·접촉 계측

GT 속도를 sim20Hz에 보간, centered5point smoothing 후 acceleration/jerk를 계산하고 양 끝 .25초를 제외했다. stationary는 병진 속도 .01m/s 미만이므로 제자리 회전도 포함한다. stop/반전은 기록 threshold 기준이다. 오래 정지한 실패의 전체 jerk가 낮아지는 것을 좋은 주행으로 해석하지 않는다. acceleration RMS/p95도 JSON/CSV에 보존했다.

{table(['단계','목표','선형 jerk RMS','선형 jerk p95','각 jerk RMS','각 jerk p95','stationary s','stop','yaw 반전','lateral 반전'],smooth)}

{table(['단계','목표','최종 |vx|,|vy|,|wz| max','최종 IK wheel max','joint wheel max','장애물 contact episodes','끝 정지'],bounds)}

기존 기준 contact는 sensor 미계측이므로 null이다. 완화 후의 최종 5개는 raw contact bag의 ground 접촉 양성 대조와 instrumented scene을 확인했다. wheel 수치는 기록된 명령과 joint 샘플의 최댓값이며 샘플 사이의 연속 최대를 증명하지는 않는다. Nav2 추정과 GT 사이 수 cm 차이가 있어 Nav2 도착이 GT15cm 이내를 뜻하지 않으며 두 오차를 그대로 기록했다.

## 재현·보존 자료와 검증

- `results.json/.csv`: baseline / conservative / relaxed를 분리한 결과 및 대표 진단. 각 단계 `final_results.json/.csv`와 `REPORT.md`도 있다.
- `sol61_20261013_0620/final_conservative/`, `relaxed_sol61_20261013/final_relaxed/`: 목표별 raw bag, launch log, GT/commands/joints, steady compute CSV, CPU/RSS, runtime verified parameters, event timeline, 설정 및 scene. 최종 YAML은 각 단계 `final_controller.yaml`이고 완화의 역할/한도는 `final_profile.yaml`에 추가했다.
- `source.patch`: controller adapter/headers/manifest/CMake/README/test만 포함한다. `experiment-scripts.tar.gz`, 각 relaxed run의 `experiment_source/`, `provenance.json`, `SHA256SUMS`, `reproduce.sh`로 실행 소스와 버전을 보존했다. git push/commit은 하지 않았다.
- 가장 작은 관련 검증: C++ yaw/arc/invalid-input tests2개, Python wheel bound/axis extrema/slew/invalid tests4개 통과. 패키지 build와 모든 최종 Gazebo runtime 검증을 완료했고, configured yaw/wheel cap의 변경 거절도 실행 중 확인했다.
- 중단 기록 `01_short_ttc`, `01_short_ttc_232`는 인계에서 무효 측정으로 지정돼 비교에서 제외했다. `v2_01_short_ttc`는 정상 완료된 대표 결과다. 기존 최종과 새 최종 모두 5목표 각1회이고 초기부터 반복 캠페인을 돌리지 않았다.
- 정리 실수: legacy RPP launch77608에 SIGINT를 보내 그 하위 friction Gazebo77630도 종료됐다. 인계에서 보존하도록 한 별도 friction process였으므로 잘못된 종료였다. `legacy_owned_processes.json`에 당시 상태를 보존했다. 이후 새 실행에서 생성한 process만 정리했으며 최종 소유 partition 잔여 검사는 `cleanup.json`에 남겼다.

재현: `bash reproduce.sh conservative` 또는 `bash reproduce.sh relaxed`는 새로운 고유 출력 디렉토리를 만들며 domain228/UUID partition/localhost discovery로 격리한다. 기존 evidence를 덮어쓰지 않는다. 실제 Full access / approval never 적용 이후 실행을 이어갔다.

Native 동작 근거: [Nav2 RPP1.1.20](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp), [Collision Monitor1.1.20](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_collision_monitor/src/collision_monitor_node.cpp), [Fortress Contact system](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/contact/Contact.cc).
'''
    (ROOT/'REPORT.md').write_text(text)
    for phase,directory in [('conservative','sol61_20261013_0620'),('relaxed','relaxed_sol61_20261013')]:
        selected=[r for r in position if r[0]==labels[phase]]
        (ROOT/directory/'REPORT.md').write_text(f'# RPP {labels[phase]} 최종 5목표\n\n도착 {arrivals[phase]}/5. 목표별 각1회이며 Nav2 판정과 GT 오차를 분리했다. 상세 진단·계산·부드러움·한계·재현 자료는 [통합 REPORT](../REPORT.md)에 있다.\n\n'+table(['단계','목표','Nav2 종료','sim초','GT XY cm','GT yaw °','Nav2 XY cm','Nav2 yaw °'],selected)+'\n')
    print('Reports saved:',ROOT/'REPORT.md')
if __name__=='__main__':main()
