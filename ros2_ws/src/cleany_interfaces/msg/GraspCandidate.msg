# Pose of the configured Cleany *_grasp_tcp in the header frame.
std_msgs/Header header
string snapshot_id
uint32 object_id
geometry_msgs/Pose tcp_pose

# Unit vector derived from the configured TCP approach axis.
geometry_msgs/Vector3 approach_direction
float32 required_opening_m
float32 grasp_depth_m
float32 score

# Original selected-object OBB for MoveIt Planning Scene registration.
DetectedObject3D target_object
