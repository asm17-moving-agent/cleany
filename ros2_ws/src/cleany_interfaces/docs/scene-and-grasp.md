# Scene와 Grasp 계약

- 원본: 패키지 msg/action/srv
- 좌표: m, 오른손 좌표계

## Snapshot·객체

- Array header: 전체 객체의 capture stamp·frame
- object_id: snapshot 내 1부터 시작하는 선택 번호
- snapshot_id: opaque identifier
- OBB: pose=중심·방향, size=local X/Y/Z 전체 길이
- confidence: [0,1]
- InspectScene: 동기 RGB-D → detector → segmenter → 3D reconstruction → TF; 취소 가능
- 빈 query: 기본 prompt. 정상 빈 장면: success=true / ERROR_NONE / objects=[]
- 오류: timeout·API·JSON·mask·depth·plane·TF·cancel·internal 구분

## Grasp 후보

- PlanGrasp 입력: snapshot/object ID, target/context cloud, OBB. 숨은 cache 의존 없음
- Cloud: 같은 nonzero capture stamp·frame
- TF: capture 시각 사용. Latest 대체 금지
- PointCloud2: XYZ float32, packed RGB float32/uint32; step·payload 검사, padded row 지원
- 후보: score 내림차순. 후보 없음은 실패; 가짜 pose 금지
- TCP: rigid rotation, det +1. Reflection 거절
- Provider·approach 설정: cleany_grasping/config

## 도달성 선택

- SelectReachableGrasp: 같은 snapshot/object/frame/OBB 후보 사용
- 검사: 양팔 IK·state validity·pregrasp/grasp 2구간 plan-only
- RobotState: 완전하고 fresh한 12관절. Goal 하나씩 처리
- 임시 ACM: terminal 전 복원 필수. 복원 실패는 성공 아님
- 결과: candidate index·arm·candidate·endpoint joint state
- Trajectory 미반환: 현재 RobotState에 종속
- Position-only IK: 목표 orientation 도달 미검증
- MuJoCo 접근 데모: gripper close·attach·lift 성공 증거 아님
- 안내: [Planning](../../cleany_moveit_config/docs/planning.md), [실행](../../../../docs/RUNNING.md)
