# Optional geometry for exactly one captured, selected object. This message
# does not change historical InspectScene/PlanGrasp/GraspCandidate schemas.
std_msgs/Header header
string snapshot_id
uint32 object_id
# Mesh vertices are local to mesh_pose, which is expressed in header.frame_id.
geometry_msgs/Pose mesh_pose
# Convex footprint of observed points, extruded to the perceived support.
# This is not simulator ground truth and does not assert hidden geometry.
shape_msgs/Mesh mesh
