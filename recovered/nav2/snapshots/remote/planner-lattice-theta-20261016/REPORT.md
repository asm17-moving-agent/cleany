# Lattice / Theta* + 동일 MPPI: 공통 8경로 주행

두 플래너를 공통 8개 목표에 각각 1회 새로 실행했다. 수동 경유점 없이 HOME에서 원래 목표 하나만 전송했다. 각 주행은 독립 world·ROS domain·Gazebo partition을 사용했다.

이번 묶음의 Nav2 도착은 Lattice 8/8, Theta* 7/8이다. Nav2가 도착한 주행들의 GT 오차는 모두 기존 30cm/10도 안이었다. Theta*의 다른 방 이동은 경로 생성 후 주행 도중 종료됐으며, MPPI 출력 rollout 충돌 거부와 후퇴 충돌 거부 로그가 기록됐다. 반복 성공률이나 플래너 전체의 확정 우열로 해석하지 않는다.

MPPI는 desktop-cleany 후속 개선의 candidate_diverse 버전이다. 동일한 컨트롤러·비용·출력 충돌 검사·진행량 감지·복구 BT를 유지했다. 두 플래너의 지도/월드/컨트롤러/실제 라이브러리 SHA256 일치 검사는 condition-audit.json에 기록했다. 플래너 자체는 stock Smac Lattice(기존 80초·5cm omni primitives)와 stock Theta* 기본 설정이다. 두 플래너의 native 경로 방향 정책은 변경하지 않았다.

휠 명령 5rad/s, 실제 footprint 47.7×65.48cm/padding0, 최종 30cm/10도를 유지했다. 목적지 의자는 기존 잠금 파일에 지정된 것만 제거했다. AMCL을 사용하고 GT는 관측에만 사용했다.

이전 Gazebo 완화 실험과 동일하게 외부 pan 정렬·CollisionMonitor·3D guard는 shadow 출력이며 실제 출력은 Nav2→휠 결합 제한→Gazebo다. MPPI 내부 footprint/출력 궤적 충돌 검사는 유지했다. 초기 Lattice 회전 도착 실행은 목표 전송 전 ROS CLI 파라미터 덤프 대기로 중단됐으며 startup-failed 디렉터리에 보존했다. 계측을 동일 서비스의 직접 조회로 바꾼 뒤 재시작했고 성능 표에는 실제 목표를 보낸 주행만 포함했다.

## GIF

[2행 × 8열, 위 Lattice / 아래 Theta*, 2배속](media/lattice-theta-2rows-8cols-2x.gif)

실제 Gazebo GT를 해당 주행 지도 위에서 재생한 GIF이며 GUI 화면 녹화가 아니다. 차체 실제 크기·방향과 이동 궤적을 표시했다. 0.4sim초마다 200ms를 재생하므로 2배속이다. 각 열은 동일 목표와 동일 배율이며 완료 후 최종 자세를 유지한다. 실패/정지 구간은 삭제하지 않았다.

## 경로별 관측 결과

| 경로 | 플래너 | Nav2 status | GT cm / ° | sim s | 실제 RTF | MPPI compute p95 ms | 정지 s | joint 최대 rad/s | 접촉 기록 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| straight | lattice | 4 | 25.12 / 0.46 | 4.43 | 0.245 | 6.69 | 0.35 | 4.411 | 0 |
| straight | theta | 4 | 10.21 / 8.53 | 7.23 | 0.254 | 5.03 | 0.35 | 4.314 | 0 |
| turn_arrival | lattice | 4 | 0.57 / 8.44 | 17.21 | 0.229 | 6.15 | 0.45 | 4.724 | 0 |
| turn_arrival | theta | 4 | 26.70 / 0.52 | 9.97 | 0.197 | 6.92 | 0.25 | 4.808 | 0 |
| chair_gap | lattice | 4 | 5.56 / 7.52 | 28.55 | 0.255 | 5.12 | 0.50 | 4.707 | 0 |
| chair_gap | theta | 4 | 6.04 / 5.88 | 28.99 | 0.233 | 12.55 | 0.30 | 5.016 | 0 |
| far_heading | lattice | 4 | 0.91 / 7.90 | 31.23 | 0.242 | 5.22 | 0.40 | 5.135 | 0 |
| far_heading | theta | 4 | 2.55 / 7.61 | 25.52 | 0.251 | 5.00 | 0.40 | 4.768 | 0 |
| distant | lattice | 4 | 7.73 / 5.95 | 60.73 | 0.113 | 20.81 | 0.45 | 5.319 | 0 |
| distant | theta | 4 | 7.62 / 8.73 | 60.79 | 0.262 | 5.88 | 0.50 | 4.950 | 0 |
| lateral | lattice | 4 | 26.09 / 0.21 | 2.44 | 0.244 | 5.47 | 0.40 | 4.153 | 0 |
| lateral | theta | 4 | 25.91 / 0.64 | 5.49 | 0.102 | 14.14 | 0.10 | 3.413 | 0 |
| narrow_seat | lattice | 4 | 5.78 / 8.38 | 14.10 | 0.210 | 7.85 | 0.40 | 4.343 | 0 |
| narrow_seat | theta | 4 | 10.28 / 7.51 | 12.00 | 0.077 | 31.88 | 0.05 | 4.765 | 0 |
| other_room | lattice | 4 | 3.27 / 7.59 | 102.13 | 0.153 | 27.49 | 1.40 | 5.402 | 0 |
| other_room | theta | 6 | 1371.06 / 90.00 | 30.87 | 0.083 | 38.96 | 0.45 | 5.075 | 0 |

Nav2 4=SUCCEEDED, 5=CANCELED, 6=ABORTED. GT 오차와 Nav2 status를 별도로 표시한다. compute는 TimedController 내부 MPPI 계산 wall 시간이며 planner 계산 시간이 아니다. planner 전용 계산 시간은 이번 계측에서 null(별도 계측 없음)이다. 접촉은 비바닥·비자체 센서 기록 수이며 0이 독립적인 비침투 증명은 아니다. 휠 명령 제한과 물리 joint 응답은 별도 수치다.

관측된 최대 휠 명령은 5.000000rad/s, 실제 joint 응답 최대는 5.402rad/s였다. 따라서 5rad/s 설정을 실제 관절 응답의 절대 상한으로 주장하지 않는다. 제어주기 누락 경고는 전체 455회이며, 병렬 실행 부하와 계측을 포함한 결과다.

각 경로 1회 관측이므로 반복 성공률이나 통계적 우위를 주장하지 않는다. 처음에는 Gazebo 2개, 사용자 추가 병렬 요청 후에는 최대 4개를 동시에 실행했다. 오래 걸리는 M1 두 주행을 별도 worker로 먼저 예약했으며, 기존 순차 worker는 해당 폴더를 만나면 중복 실행하지 않는다. 실제 RTF와 연산시간은 당시 공유 부하의 영향을 받으므로 플래너 고유 연산 성능 순위로 해석하지 않는다. 요청 RTF는 2이나 실제 RTF는 위 관측값을 사용한다. 실패한 조기 종료의 시간·낮은 jerk는 개선으로 해석하지 않는다. 점수나 합격 기준은 추가하지 않았다.

전체 정확도·시간·연산 median/p95/p99·jerk·정지·반전·휠·접촉 수치는 [CSV](comparison.csv)와 [JSON](comparison.json)에 있다.

개별 경로 GIF:

- [1. straight](media/01-straight-pair-2x.gif)
- [2. turn_arrival](media/02-turn_arrival-pair-2x.gif)
- [3. chair_gap](media/03-chair_gap-pair-2x.gif)
- [4. far_heading](media/04-far_heading-pair-2x.gif)
- [5. distant](media/05-distant-pair-2x.gif)
- [6. lateral](media/06-lateral-pair-2x.gif)
- [7. narrow_seat](media/07-narrow_seat-pair-2x.gif)
- [8. other_room](media/08-other_room-pair-2x.gif)

## 종료한 실패 주행

- theta / other_room: 최종 GT [-5.522977666572982, -6.058981187917169, -4.083332402072247e-05], 목표까지 13.71m. 실제 로그의 오류/복구 이벤트는 comparison.json의 failures에 보존했다. 경로 생성 이후 주행 실패와 경로 탐색 실패를 혼동하지 않는다. 원인 확정을 위한 추가 튜닝/재실험은 하지 않았다.

원격 원자료: /home/donglee/workspace/cleany/planner-lattice-theta-20261016. 기존 소스·설정·산출물은 보존했고 git push/물리 로봇 검증은 하지 않았다.
