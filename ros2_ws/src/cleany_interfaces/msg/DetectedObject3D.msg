# 사용자에게 표시하는 snapshot 내 번호이며 1부터 시작합니다.
uint32 object_id

# Normalized semantic type and original detector class, as in DetectedObject2D.
string label
string detector_label
# Detector confidence in [0, 1], not semantic classification confidence.
float32 confidence

# 방향성 경계 상자의 중심, 방향과 로컬 축별 전체 길이(m)입니다.
# 두 필드 모두 상위 DetectedObject3DArray의 header 좌표계를 사용합니다.
geometry_msgs/Pose obb_pose
geometry_msgs/Vector3 obb_size

string track_id
string tracking_state
