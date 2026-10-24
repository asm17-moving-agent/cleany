# Nav2 표준 Smac Lattice 전환 확인

StraightFirstPlanner를 Nav2 표준 SmacPlannerLattice로 교체한 Axis/Omni프로필과 설치된 primitive경로 해석을 별도 checkout에 적용했다. 실제 MPPI+Gazebo시험은 기존 최종 MPPIoverlay를 사용하고 표준 planner를 직접 로딩했다. main은 navigation정리 작업과 여러 dirty변경이 있어 수정하거나 merge하지 않았다. sourcecheckout은 /home/ehdrms/workspace/cleany/standard-nav2-planner, branch codex/standard-nav2-planner-20261014, 비교를 실행했던 b3ca816기반이다. Gitpush없음.

Omni는 설치된 nav2_smac_planner의5cm/0.5m Omni primitives를 수정 없이 사용한다. Axis는 기존 정책과 같이 stockdiff에서직선/제자리회전만선택한다. 커스텀 direct/heading-fixed 검색과 별도 swept-cell검사는 standardplugin에서 실행되지 않는다. 실제 nativeplanner parameter와 로딩로그를 runtime-validation-v4/2_chair_gap에 보존한다.

지도는 기존2.5cm 원본의2x2 cell을 보수적으로 합쳐5cm로 맞췄다. 기존inputworld/목표의자제거/footprint/.02padding/.8inflation은 보존한다. map해상도와 planner가 바뀌므로 이전controller결과와 직접적인 알고리즘단독 효과로 비교하지 않는다. primitive metadata만 바꾸지 않았다. sharedcommon과 기존실험원자료를 수정하지 않았다.

- 정적 경로 생성:8개 중7개 성공. M1다른방은5초planningbudget에서 경로생성중단. 실패를 숨기거나margin을 줄이지 않았다.
- 실제 MPPI+표준planner 의자사이:Nav2SUCCEEDED,39.983sim초/114.502wall초, GT10.9447cm/8.2384도.
- navigationbuild2packages완료. Python118passed3skipped, 최종runtimeconfig25passed. 별도Gazeboheadless주행1개만수행했다. 실제로봇검증없음.
- v1은PillowAPI호환오류, v2/v3는5cm지도변경후기존maphash검증단계에서중단됐다. 세시도모두주행목표를전송하지않았다. 원본2.5cmmaphash와derived5cmmaphash를별도기록한 뒤v4에서주행했다.
- 외부safetybypass는기존Gazebo완화프로필 그대로이며 물리로봇설정으로배포하지않았다.

전체JSON은REPORT.json, 정적path/preflight파라미터는preflight/, 실제bag/log/commands/GT/results는runtime-validation-v4/2_chair_gap/이다.
