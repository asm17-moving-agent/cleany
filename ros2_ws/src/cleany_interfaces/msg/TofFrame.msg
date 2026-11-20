# 펌웨어 원래 zone 순서입니다. 무효 zone은 빈 공간의 근거가 아닙니다.
# stamp는 ROS 시계 영역에서 취득 시각을 보수적으로 추정합니다.
std_msgs/Header header

# false이면 펌웨어, 전송 또는 session 오류 이후 캐시된 거리를 무효화합니다.
bool sensor_ready
uint32 sequence
uint64 device_time_us
float32[64] distance_m
float32[64] sigma_m
uint8[64] target_status
uint8[64] target_count
bool[64] valid
