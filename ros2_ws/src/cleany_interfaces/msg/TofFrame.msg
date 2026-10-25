# Native firmware zone order. Invalid zones are never free-space evidence.
# Stamp conservatively estimates acquisition time in the ROS clock domain.
std_msgs/Header header
uint32 sequence
uint64 device_time_us
float32[64] distance_m
float32[64] sigma_m
uint8[64] target_status
uint8[64] target_count
bool[64] valid
