# Raw cumulative signed quadrature counts from the motor controller.
# Order: M1 front-left, M2 front-right, M3 rear-right, M4 rear-left.
# No direction correction, unwrapping, or ticks-to-radians conversion.
# Stamp is the Jetson ROS clock at HTTP response receipt, NOT MCU sample time.
std_msgs/Header header
int32[4] ticks
# Measured with a monotonic host clock, including response body transfer.
float64 round_trip_time_sec
# False for legacy firmware. Odometry requires valid MCU metadata.
bool has_mcu_time
string boot_id
uint32 sample_seq
# Monotonic MCU microseconds at the atomic counter snapshot, not Unix time.
uint64 sample_time_us
