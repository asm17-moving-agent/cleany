# Snapshot-local number presented to users. Numbering starts at 1.
uint32 object_id

# Normalized semantic type and original detector class, as in DetectedObject2D.
string label
string detector_label
# Detector confidence in [0, 1], not semantic classification confidence.
float32 confidence

# Oriented bounding-box center/orientation and full local-axis extents in m.
# Both fields use the enclosing DetectedObject3DArray header frame.
geometry_msgs/Pose obb_pose
geometry_msgs/Vector3 obb_size

string track_id
string tracking_state
