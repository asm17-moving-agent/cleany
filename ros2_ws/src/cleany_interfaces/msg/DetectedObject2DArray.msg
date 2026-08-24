# Capture timestamp and RGB optical frame shared by all detections. Detections
# with valid depth are ordered by target-frame distance, detector confidence,
# then their original detector order. Invalid-depth detections follow them.
std_msgs/Header header

# Opaque identifier used by the later selected-object inspection request.
string snapshot_id

DetectedObject2D[] detections
