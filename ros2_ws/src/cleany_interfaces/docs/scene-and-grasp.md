# Scene와 Grasp 계약

필드 원본은 이 패키지의 msg/action/srv다. 좌표는 m, 회전은 오른손 좌표계다.
DetectedObject3DArray의 header가 모든 객체의 capture stamp와 frame을 소유한다.
object_id는 snapshot 안의 1부터 시작하는 선택 번호, snapshot_id는 opaque identifier다.
OBB pose는 중심·방향, size는 OBB local X/Y/Z의 **전체 길이**다. confidence는 [0,1]이다.

InspectScene은 동기화한 RGB-D에 detector → segmenter → 3D reconstruction → TF 변환을 한 번 수행하는 cancel 가능한 action이다.
빈 query는 node 기본 prompt다. 정상 빈 장면은 success=true, ERROR_NONE, 빈 objects다.
RGB-D timeout·API·JSON·mask·depth·plane·TF·cancel·internal 오류를 구분한다.

## 후보 생성

PlanGrasp는 snapshot/object ID, target/context cloud와 OBB를 직접 전달한다. 숨은 perception cache를 참조하지 않는다.
Cloud는 동일한 nonzero capture stamp와 frame을 가져야 한다. TF는 capture 시각을 사용하며 latest로 대체하지 않는다.
PointCloud2의 XYZ float32와 packed RGB, point_step/row_step/payload를 검증하고 organized/padded row를 처리한다.
후보는 score 내림차순이다. 후보 없음은 실패이며 가짜 pose를 만들지 않는다.
TCP 회전은 rigid rotation(det +1)이어야 하며 reflection은 거절한다.
Provider·approach 설정은 cleany_grasping/config가 관리한다.

## 도달 가능 후보 선택

SelectReachableGrasp는 같은 snapshot/object/frame/OBB의 후보를 받는다.
양팔 IK·state validity·pregrasp/grasp 2구간 **plan-only** 검증을 수행한다.
완전하고 fresh한 12관절 RobotState를 사용하고 한 번에 goal 하나만 처리한다.
임시 ACM은 terminal 결과 전에 복원해야 한다. 복원 실패는 성공이 아니다.
성공 결과는 후보 index·arm·candidate·endpoint joint state다.
Trajectory는 현재 RobotState에 종속되어 결과에 포함하지 않는다.
현재 position-only IK는 목표 orientation 도달을 검증하지 않는다.
MuJoCo 접근 데모는 gripper close·attach·lift의 물리 성공을 증명하지 않는다.

[Planning](../../cleany_moveit_config/docs/planning.md), [실행](../../../../docs/RUNNING.md).
