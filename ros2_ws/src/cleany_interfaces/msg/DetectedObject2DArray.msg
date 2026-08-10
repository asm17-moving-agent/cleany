# Capture timestamp and RGB optical frame shared by all detections.
std_msgs/Header header

# Opaque identifier used by the later selected-object inspection request.
string snapshot_id

DetectedObject2D[] detections
