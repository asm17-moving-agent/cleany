# TEB 최적화 및 검증 완료 — 2026-10-13

보수 공통 제한과 별도 승인된 Gazebo 완화 조건을 분리해 최적화하고, 각 단계에서 설정을 동결한 뒤 공통 다섯 목표를 각각 한 번 평가했다. Nav2 도착과 GT 15cm/10° 관측은 별도로 집계한다. 통합 점수나 합격기준은 만들지 않았다.

| 조건 | Nav2 도착 | 도착 중 GT 범위 관측 | compute p95 범위 ms | joint wheel 최대 rad/s | 물리 접촉 episodes |
| --- | --- | --- | --- | --- | --- |
| conservative | 2/5 | 1/5 | 24.97–33.77 | 4.988 | null (센서 없음) |
| gazebo_relaxed | 5/5 | 5/5 | 8.32–14.42 | 4.990 | 0 |

보수 단계는 min_obstacle_dist=0.05m만 최종 native 변경으로 채택했다. 원래 기준선의 Nav2 도착은1/5였다. 보수 단계에서 pan 방향 전환·깊이 freshness 대기와 progress 실패가 남았다. 완화 단계는 이 별도 전달 가드를 우회하고 wheel envelope·가속 제한, watchdog, TEB search/time discretization 및 controller 주기를 함께 바꿨다. 따라서 두 단계 차이를 TEB-only 개선으로 해석하거나 다른 controller와 그대로 순위 비교하지 않는다.

완화 단계의 실제 경로는 Nav2 → uniform wheel-envelope scaling → wheel별 slew를 제한하는 convex twist 보간 → cmd_vel → Gazebo guard다. r=.0635m, wheelbase=.35m, separation=.6038m, 물리 wheel cap5rad/s를 유지한다. 수치 여유를 위해 출력 cap4.99, wheel 가속8rad/s², 출력20Hz를 채택했다. 기존 Gazebo body축 가속 제한을 사용한 초기 pilot에서 joint5.302rad/s 초과가 관측됐다. 독립 축 보간이 결합 envelope를 벗어날 수 있어 이를 결합 limiter로 대체했다. 이 actuator 변경은 완화 조건의 공정한 controller 비교에서 공통 적용해야 한다. 실제 collision geometry, padded footprint, 내부 feasibility, map/world와 목표는 유지했다. 물리 로봇 적용 설정이 아니다.

기존240sim초는 완화 단계의 소프트 표식이며 hard watchdog은1200sim/1800wall초다. goal checker는 그대로15cm/10° stateful이다. RTF 목표1, 실제 주기·RTF·wall compute budget·CPU/RSS·GT jerk·정지/반전·오류/예외·접촉 및 wheel 명령/joint 값은 아래 원시 결과에 남겼다. 보수 단계의 접촉은 계측되지 않았으므로0으로 간주하지 않는다. 완화 단계 접촉은 full rosbag2 물리 collision-pair episode이며 floor positive control을 검증했다.

| 목표 | 보수 Nav2 / sim s / GT cm,° | 완화 Nav2 / sim s / GT cm,° |
| --- | --- | --- |
| straight | aborted / 115.666 / 47.80, 13.73 | arrived / 9.718 / 7.32, 7.59 |
| turn_arrival | arrived / 86.972 / 15.68, 4.53 | arrived / 22.902 / 5.84, 1.59 |
| chair_gap | aborted / 155.458 / 142.97, 2.87 | arrived / 22.470 / 6.62, 3.47 |
| far_heading | arrived / 85.056 / 10.65, 5.65 | arrived / 33.590 / 3.86, 1.03 |
| distant | timeout / 240.000 / 401.94, 14.45 | arrived / 59.934 / 6.71, 5.52 |

실패/timeout의 GT 오차는 종료 시 잔여 거리·방향이며 도착 정확도가 아니다. 각 목표 한 회이므로 일반 성공률을 추정하지 않는다. 기존/새 TEB는 모두-O3 -DNDEBUG였고 Release 빌드 성능 개선이라고 주장하지 않는다. 다른 CPU 호스트와 절대 compute/CPU 순위를 비교하지 않는다.

- [conservative 상세 REPORT](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/REPORT.md): 원인, 대표 pilot, 최종 정확도·시간, median/p95/p99, budget/예외, CPU/RSS, GT jerk, 접촉·제한 검증.
- [conservative JSON](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/results.json), [CSV](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/results.csv), [최종 native YAML](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/final_controller.yaml), [재현 절차](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/REPRO.md). 원시 final 목표별 bag/log/CSV/JSON, 호환·진단·runner patch, source/build archive 및 환경 메타데이터를 함께 보존했다.
- [gazebo_relaxed 상세 REPORT](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/REPORT.md): 원인, 대표 pilot, 최종 정확도·시간, median/p95/p99, budget/예외, CPU/RSS, GT jerk, 접촉·제한 검증.
- [gazebo_relaxed JSON](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/results.json), [CSV](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/results.csv), [최종 native YAML](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/final_controller.yaml), [재현 절차](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/REPRO.md). 원시 final 목표별 bag/log/CSV/JSON, 호환·진단·runner patch, source/build archive 및 환경 메타데이터를 함께 보존했다.

[단계별 통합 JSON](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/results_by_stage.json), [CSV](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/results_by_stage.csv), [완화 artifact 검증](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/artifact_checks.json).

![최종 GT 경로](/home/donglee/workspace/cleany/controller-optimization-20261013/teb/final_trajectories.png)

Full access에서 기존 소유 시뮬레이션을 확인하고 진행했다. 소유 partition/PID만 정리했고 다른 작업의 프로세스·파일과 기존 dirty checkout을 보존했다. dedicated Cleany source는 pinned b3ca816ba420452ee7960837bc16ec74148e07b0의 clean 상태다. push하지 않았다. cleanup.json에 각 단계의 소유 프로세스가 남지 않았음을 기록했다.
