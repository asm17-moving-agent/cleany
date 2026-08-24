# Common capture timestamp and coordinate frame for every object.
std_msgs/Header header

# Opaque identifier used to correlate inspection and grasp-planning requests.
string snapshot_id

DetectedObject3D[] objects
