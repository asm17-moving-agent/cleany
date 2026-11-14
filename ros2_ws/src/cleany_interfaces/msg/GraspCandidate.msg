# header 좌표계에서 설정된 Cleany *_grasp_tcp의 자세입니다.
std_msgs/Header header
string snapshot_id
uint32 object_id
geometry_msgs/Pose tcp_pose

# 설정된 TCP 접근 축에서 구한 단위 벡터입니다.
geometry_msgs/Vector3 approach_direction
float32 required_opening_m
float32 grasp_depth_m
float32 score

# 후보 header 좌표계로 변환한 선택 물체의 OBB입니다.
DetectedObject3D target_object
