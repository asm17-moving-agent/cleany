# 고정 크기 micro-ROS 계약 v2입니다. 모든 휠 배열 순서는 FL, FR, RL, RR입니다.
uint8 PROTOCOL_VERSION=2
uint8 STOP=0
uint8 ENABLE=1
uint8 VELOCITY=2
uint8 protocol_version
uint32 sequence
# MCU 단조 증가 시계의 마감 시각이며 STOP에서는 무시합니다. ROS epoch를 사용하지 않습니다.
uint64 valid_until_us
uint8 mode
float32[4] velocity_rad_s
