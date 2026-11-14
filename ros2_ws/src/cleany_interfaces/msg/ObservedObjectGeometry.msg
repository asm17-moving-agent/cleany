# 촬영하고 선택한 물체 정확히 하나의 선택적 형상입니다. 이 메시지는
# 기존 InspectScene/PlanGrasp/GraspCandidate 스키마를 바꾸지 않습니다.
std_msgs/Header header
string snapshot_id
uint32 object_id
# 메시 정점은 mesh_pose의 로컬 좌표이며 mesh_pose는 header.frame_id로 표현합니다.
geometry_msgs/Pose mesh_pose
# 관측 점의 볼록 footprint를 인식된 지지면까지 확장한 형상입니다.
# 시뮬레이터 참값이 아니며 가려진 형상을 보장하지 않습니다.
shape_msgs/Mesh mesh
