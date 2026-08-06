# Snapshot-local number presented to users. Numbering starts at 1.
uint32 object_id

# Detector label and normalized confidence in [0, 1].
string label
float32 confidence

# Oriented bounding-box center/orientation and full local-axis extents in m.
# Both fields use the enclosing DetectedObject3DArray header frame.
geometry_msgs/Pose obb_pose
geometry_msgs/Vector3 obb_size
