# TEB / RPP 정식 10회 측정

동결된 설정으로 각 컨트롤러에 동일한 다섯 목표를 한 번씩 실행했다. RPP는 도착 3개, 제한 시간 취소 1개, 중단 1개였다. TEB는 도착 1개, 중단 4개였다. 이 상태는 관측 기록이며 합격·불합격 판정이나 알고리즘 일반 성능 순위가 아니다.

| 컨트롤러 | 목표 | 종료 상태 | 주행 sim 초 | 위치 거리 cm | 방향 차이 ° |
|---|---|---|---:|---:|---:|
| RPP | straight | 도착 | 16.850 | 14.70 | 1.66 |
| RPP | turn_arrival | 도착 | 42.470 | 12.97 | 7.81 |
| RPP | chair_gap | 240초 제한 취소 | 240.046 | 138.29 | 92.90 |
| RPP | far_heading | 도착 | 55.930 | 11.75 | 7.30 |
| RPP | distant | 중단 | 66.978 | 327.45 | 89.36 |
| TEB | straight | 중단 | 42.566 | 66.32 | 22.15 |
| TEB | turn_arrival | 도착 | 125.044 | 14.36 | 5.44 |
| TEB | chair_gap | 중단 | 130.996 | 276.85 | 27.18 |
| TEB | far_heading | 중단 | 26.118 | 524.75 | 133.01 |
| TEB | distant | 중단 | 191.178 | 565.59 | 3.17 |

도착 행만 GT 도착 오차다. 나머지 행은 종료 뒤 정지 상태에서 남은 목표 거리와 방향 차이다. `completed`는 측정 완료를 뜻하며 도착 여부는 `arrived` / `terminal_status`에 별도로 기록했다.

## 제어 호출 시간

같은 Ryzen 5 9600X 호스트에서 병렬 Gazebo 실행 중 측정한 inner controller steady-wall 시간이다. 성공 반환과 예외를 모두 포함한 값이며 두 모집단별 통계도 JSON에 보존했다. 파일 기록 시간은 측정 구간 밖이고, 알고리즘 내부 시각화·로그 및 스케줄링 시간은 포함된다. Nav2 전체 주기 시간이 아니다.

| 목표 | RPP p95 / p99 ms | TEB p95 / p99 ms |
|---|---:|---:|
| straight | 0.227 / 0.685 | 58.928 / 71.267 |
| turn_arrival | 0.244 / 0.860 | 64.690 / 74.301 |
| chair_gap | 0.155 / 0.819 | 61.040 / 69.271 |
| far_heading | 0.240 / 0.416 | 65.045 / 73.982 |
| distant | 0.145 / 0.239 | 60.703 / 68.685 |

목표 RTF는 1, 실제 RTF는 0.941–0.975였다. TEB의 정식 호출 p95는 모두 20Hz의 50ms wall 예산을 넘었다. 이는 해당 설정·호스트·병렬 부하의 관측으로, 다른 호스트의 DWB/MPPI와 직접 계산 성능 순위를 매기지 않는다. 단독 실행 보조 측정은 정식 데이터와 분리한다.

동일 호스트의 단독 straight 보조 실행도 완료했다. DWB와 RPP는 도착, TEB는 중단했다. 아래는 각 한 회의 참고 계측이며 정식 10회에 추가하지 않는다. TEB의 단독 p95도 50ms를 넘으므로 정식 계산 시간 초과를 병렬 실행만의 영향이라고 단정할 수 없다.

| 보조 컨트롤러 | 호출 p95 ms | 호출 p99 ms |
|---|---:|---:|
| DWB | 15.444 | 16.359 |
| RPP | 0.158 | 0.214 |
| TEB | 51.693 | 57.261 |

## 안전·해석 제한

각 휠의 실제 피드백 최댓값은 RPP 3.414rad/s, TEB 4.988rad/s로 5rad/s 이하였고, 열 개 실행 모두 마지막 최종 명령과 실제 속도가 0이었다. 샘플 기반 보수적 정적 footprint 투영에서 겹침은 관측되지 않았다. 정적 최소 간격 범위는 RPP 0.155–0.297m, TEB 0.155–0.208m였다. 실제 물리 접촉 센서는 없으므로 접촉 횟수는 **미계측**이다.

`STOP_OBSTACLE` 대기와 `STOP_UNAVAILABLE`, 처리 예산 초과를 분리했다. 특히 RPP chair_gap의 약 206.858sim초는 `Command timeout`이었다. 이를 장애물의 이동을 기다린 시간이나 센서 지연으로 해석하지 않는다. 2D `/collision_monitor_state`는 기록된 메시지가 없으며, 기본값 0은 안전 개입 0회라는 근거가 아니다. 일부 bag에는 `/plan`도 없어 경로 재구성에 제한이 있다.

Humble RPP에는 native 병진 가속도 제한과 경로 추종 각속도 상한이 없다. 최종 공통 guard의 속도/각속도 제한은 적용했지만 공유 velocity smoother는 없었다. 따라서 네 컨트롤러가 동일한 native 가속도 제한을 보장했다고 주장하지 않는다. TEB는 stock이 아니라 기록된 TF/cleanup 호환 패치를 적용한 humble-devel 빌드다.

Jerk는 GT 속도를 sim-time 20Hz에 보간하고 중심 5점 이동평균 후 두 번 미분했다. 양 끝 0.25초는 제외했다. 오래 정지한 실패 사례의 전체 주행 jerk가 낮을 수 있으므로 이를 좋은 주행으로 평가하지 않는다. `stationary_s`는 병진 속도 0.01m/s 미만이며 제자리 회전도 포함한다. 각 목표 한 회이고 전체 Gazebo/AMCL 확률성의 동일 seed는 보장하지 않는다.

## 재현 자료

정식 결과는 `remote_results.csv` / `.json`, 보조 결과는 `auxiliary_results.csv` / `.json`이다. 정식 원본 디렉터리는 `rpp_rtf1/`, `teb_rtf1/`이며 각 실행에 bag, 설정, GT/명령/휠 피드백, 센서 수신 시각, 안전 진단, compute CSV, CPU/RSS 및 파생 분석을 보존했다. CPU는 한 코어 100% 기준이라 멀티스레드 TEB에서 100%를 넘을 수 있다.

소스 HEAD `b3ca816ba420452ee7960837bc16ec74148e07b0`, physics dt 0.002s, 공통 목표·지도·costmap·planner·BT·안전 파이프라인을 사용했다. `environment.json`, `STATUS.md`, `teb-compat.patch`, `teb-compatible-source.tar.gz`, `teb-build-logs.tar.gz`, `measurement_ws/src/cleany_benchmark/`에 버전·빌드·패치·측정 구현을 보존했다. 기존 사용자 worktree는 수정하지 않았다.

최종 검증: 정식 10개와 보조 3개 모두 원시 기록·분석을 보존했고 종료 명령/GT 속도 0 및 휠 제한을 확인했다. 실험에서 띄운 Gazebo, Nav2 및 runner 프로세스는 종료했다. 기존 Distrobox와 사용자 변경 파일은 남겨 두었다.
