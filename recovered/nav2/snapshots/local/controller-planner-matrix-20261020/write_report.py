from pathlib import Path
import json
R=Path(__file__).resolve().parent
D=json.loads((R/'comparison.json').read_text());S=D['summary'];rows=D['runs']
labels={'dwb_lattice':'DWB + Lattice','dwb_theta':'DWB + Theta*','mppi_lattice':'MPPI + Lattice','mppi_theta':'MPPI + Theta*'}
def f(v,d=2):return 'null' if v is None else f'{v:.{d}f}'
headline=' / '.join(labels[s['combination']]+f" {s['nav2_arrivals']}/8" for s in S)
lines=['DWB / MPPI × Lattice / Theta*: 공통 8경로 비교','', '관측 Nav2 도착: '+headline+'. 현재 보존된 튜닝 프로파일과 해당 시뮬레이션에서 얻은 결과이며 일반적인 컨트롤러 우열이나 반복 성공률은 아니다.', '',
'신규 DWB 16주행(2026-10-20)과 보존된 MPPI 16주행(2026-10-16)을 비교했다. 각 조합·경로당 최종 비교 표본1개이며 반복 성공률을 측정한 실험은 아니다. 수동 경유점을 추가하지 않았다. Nav2 status와 GT 오차를 별도 기록하고 합격 기준/통합점수는 만들지 않았다.','',
'조건: 공통 HOME→8개 목표, 잠금된 18F full 월드와 잠금 파일에서 지정한 목적지 의자만 제거, 동일 지도·복구 BT(재시도4회)·SimpleProgressChecker(0.10m/20sim초)·failure_tolerance0.5sim초·30cm/10°·20Hz 설정. AMCL로 위치 추정하고 Gazebo GT는 계측에만 사용. footprint47.7×65.48cm, padding0. 휠 최종 출력 한도5rad/s, DWB 후보 한도4.99rad/s. 물리 joint 응답 최대는 별도 기록한다.','',
'Lattice는 기존 MPPI 비교에 쓰인 stock144 primitives, 5cm해상도, 반경 설정50cm를 보존했다. 이후 공식 Generator로 만든 반경10cm primitive 비교가 아니다. Theta*도 기존 비교의 기본 설정을 사용했다. DWB는 보존된 튜닝 소스44c083abe2ebeb0077879e4419660b15fe4dd91b 및 당시 프로파일을 복원·빌드했다. 휠제한/overshoot/target 관련 회귀 검사5개 통과. FollowPath.stateful은 런타임에서 선언되지 않는 비활성 설정임을 기록하고 나머지 프로파일은 실제 서비스 readback으로 확인했다.','',
'동일한 원격 CPU 호스트지만 MPPI와 DWB의 실행 날짜·동시 실행 수·서버 부하가 다르다. 신규 주행 중 기존 Isaac GPU 학습은 그대로 실행 중이었다. 연산시간·RTF는 관측값으로 제시하며 컨트롤러 고유 연산성능 순위로 해석하지 않는다. 요청 RTF2와 측정 실제RTF를 구분한다.','',
'신규 주행은 처음2개, 한때4개를 병렬 실행했다. CPU 포화/제어주기 미달을 관측한 뒤 추가로 시작한 다른 방2주행을 수동 중단하고2개로 줄였다. 그 구간에 걸린 DWB+Lattice 2번과 DWB+Theta* 1번도 결과와 무관하게 부하 영향 표본으로 분리한다고 재실행 전에 선언하고, campaign 종료 후 max2sim 상태에서 처음부터 재실행했다. load-pilot의4개 기록은 보존하되 성능 표에서 제외했다. 최종 비교32행 중DWB16행은 모두 단독~2개 병렬 실행 조건이다. 반복 성공률 표본을 늘린 것으로 해석하지 않는다.', '',
'안전 우회: Gazebo 한정으로 외부 pan 정렬/CollisionMonitor/3D guard는 shadow 출력. 실제 출력은 Nav2→결합 휠 제한→Gazebo. 컨트롤러 내부/코스트맵 충돌 검사는 유지했다. 물리 로봇 실행 및 git push 없음.','',
'조합별 전체 관측 (도착=Nav2 SUCCEEDED, GT범위는 도착 여부와 독립)','조합 | Nav2 도착 | 최종GT위치30cm내 | 최종GT30cm/10° 모두 범위 | 근접 각도미정렬 체류합(sim s, 실패포함) | 비바닥/비자체 접촉 기록 | wheel command / joint 최대(rad/s)']
for s in S:lines.append(f"{labels[s['combination']]} | {s['nav2_arrivals']}/8 | {s['gt_xy_within_30cm']}/8 | {s['gt_within_tolerance']}/8 | {f(s['total_near_goal_unaligned_sim_s'])} | {s['non_floor_contact_records']} | {f(s['max_wheel_command_rad_s'],3)} / {f(s['max_wheel_joint_rad_s'],3)}")
lines+=['', '접촉 세부: DWB+Theta* 의자 사이에서 front_right_wheel_link의 mecanum_contact와 demo_desk_36의 desk_envelope_collision 사이 접촉 센서 기록7개가 관측됐다. 동일 collision-envelope 쌍의 메시지 기록 수이며 독립 충돌7회를 의미하지 않는다. 다른 조합은 관측된 비바닥/비자체 접촉 기록0개였다.', '', '경로별 예외: 8번 다른 방은 DWB+Theta*가188.79sim초에 도착했으며 MPPI+Theta*는ABORTED였다. 따라서 모든 경로에서 한 컨트롤러가 우세하다고 해석하지 않는다. DWB는 목표 근처 각도미정렬 시간이 길었고, 모든 실패가 경로 탐색 실패였던 것은 아니다.', '', '부하 재실행의 결과 변화: DWB+Lattice 2번은 기존 부하 기록SUCCEEDED→새 기록ABORTED, DWB+Theta* 1번은ABORTED→SUCCEEDED였다. 사전 선언대로 새 기록을 채택했고 이전 기록도 보존했다. 부하뿐 아니라 AMCL/초기경로 변동이 함께 있을 수 있어 부하의 인과 효과나 반복 성공률로 주장하지 않는다. 수치 전후 비교는 load-sensitivity.json에 있다.', '', '4개 조합이 모두 도착한 동일 경로만의 비교', '대상: '+(', '.join(S[0]['common_arrival_routes']) or '없음. 공통 평균은 계산하지 않고 null로 기록함'), '아래 값은 경로별 값의 산술평균이다. jerk는 경로별 RMS를 평균하며, 전체 샘플을 합쳐 계산한 RMS가 아니다. compute p95는 경로별 p95의 평균이며 pooled p95가 아니다.','조합 | sim초 | GT cm / ° | 선형jerk RMS / p95(m/s³) | 각jerk RMS(rad/s³) | 정지초 | compute median/p95/p99(ms) | 실제RTF']
for s in S:
 def v(k,d=2):return f(s['common_routes_mean_'+k],d)
 lines.append(f"{labels[s['combination']]} | {v('sim_s')} | {v('gt_xy_cm')} / {v('gt_yaw_deg')} | {v('linear_jerk_rms')} / {v('linear_jerk_p95')} | {v('angular_jerk_rms')} | {v('stationary_s')} | {v('compute_median_ms')}/{v('compute_p95_ms')}/{v('compute_p99_ms')} | {v('actual_rtf',3)}")
lines+=['','경로별 원자료 요약','경로 | 조합 | Nav2 status | GT cm / ° | sim초 | jerk RMS 선형/각 | 정지초 | 반전 yaw/횡 | compute median/p95/p99(ms) | 실제RTF']
for r in rows:lines.append(f"{r['goal_index']+1}.{r['goal_name']} | {labels[r['combination']]} | {r['nav2_status']} | {f(r['gt_xy_cm'])}/{f(r['gt_yaw_deg'])} | {f(r['sim_s'])} | {f(r['linear_jerk_rms'])}/{f(r['angular_jerk_rms'])} | {f(r['stationary_s'])} | {r['yaw_reversals']}/{r['lateral_reversals']} | {f(r['compute_median_ms'])}/{f(r['compute_p95_ms'])}/{f(r['compute_p99_ms'])} | {f(r['actual_rtf'],3)}")
lines+=['','상태:4=SUCCEEDED,5=CANCELED,6=ABORTED. 시간 제한 취소 여부는 CSV timeout 열에 별도 표시. 조기 종료 및 정체 때문에 낮은 시간/jerk가 나온 경우를 개선으로 해석하지 않는다. 경로별 정지 시간·반전·남은 거리·status를 함께 본다.','',
'호환성 관찰: 이 DWB는 기존 튜닝된 GoalDirectedController/PathOrientationCritic 프로파일이다. PathOrientationCritic는 가장 가까운 path pose의 yaw를 비용으로 사용한다. 보존된 Theta* 직선 경로의 중간 pose yaw는0이며 마지막 pose만 목표yaw1.5708이다. 따라서 이 프로파일과 Theta*의 방향 표현이 불필요한 회전을 유도할 가능성이 있다. 실제로 첫 주행에서 GTyaw1.57→약0.3 뒤 재정렬이 관측됐다. 코드/경로 표현에 근거한 유력한 호환성 설명이며 critic을 바꾼 인과 재실험으로 확정한 결과는 아니다.', '',
'근접 각도미정렬 체류는 GT목표거리30cm이내이면서 방향오차10°밖인 샘플 구간의 sim시간 합이다. 기록된 GT의 인접 샘플 시각 차이를 왼쪽 샘플 상태로 분류한 근사치이며 실패 주행도 포함한다. 새로운 합격 기준이 아니라 기존 도착 허용 오차를 이용한 진단값이다.', '',
'정지 기준은 GT 선속도0.01m/s 및 |각속도|0.02rad/s 미만이다. 아주 느린 비영 회전도 포함되며 완전한 무동작 시간과 같지 않다. yaw 반전은0.02rad/s, GT vy 반전은0.01m/s deadband를 적용했다.', '',
'정렬 계측 예: DWB+Lattice 의자 사이의 navigation25~60sim초는 GT 목표 거리최대0.283m인 구간이다. generator 입력 측정각속도 median0, command dt median0.009초, 방출각속도 median0.0054rad/s였고 GTyaw는0.453→0.626rad로 매우 느리게 변했다. alignment-diagnostic.json에 근거를 남겼다. 낮은RTF·elapsed-time 가속 예산·속도피드백/critic의 상호작용 가능성이 있으나, 이 실험에서 원인을 분리하거나 튜닝한 것은 아니다.', '',
'측정: '+D['jerk_method'],'compute: '+D['compute_scope'],'제어주기: '+D['command_interval_scope'],'접촉: '+D['contact_scope'],'',
'GIF: media/controller-planner-4rows-8cols-2x.gif. 위부터 DWB+Lattice / DWB+Theta* / MPPI+Lattice / MPPI+Theta*. 열은 straight,turn_arrival,chair_gap,far_heading,distant,lateral,narrow_seat,other_room. 보라색 점선은 실제 초기 global plan, 색 실선은 실제 GT 이동 흔적이며, 실제 차체 크기와 방향을 표시했다. 실제 Gazebo GT를 지도 위에서 재생한 것으로 GUI 녹화는 아니다. 0.4sim초당200ms로 2배속이며 완료 후 최종 자세 유지, 실패/정지 구간 보존. 개별 경로 GIF8개도 media/에 있다.','',
'comparison.csv/JSON:32개 주행 전체. summary.csv:공통 도착 경로의 동등 가중 평균. condition-audit.json:공통 입력 및 실제 runtime 설정 확인. 원격 원자료·rosbag는 /home/donglee/workspace/cleany/cleany-data/runs/controller-planner-matrix-20261020. 신규 source/build/probe 및 startup-failed 진단도 보존. 기존 MPPI 데이터는 수정하지 않고 필요한 원자료를 baseline-snapshot에 복사하고 해시 manifest를 보존했다.']
(R/'REPORT.txt').write_text('\n'.join(lines)+'\n')
print(R/'REPORT.txt')
