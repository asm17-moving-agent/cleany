# Fixed-size micro-ROS contract v1. All wheel arrays: FL, FR, RL, RR.
uint8 PROTOCOL_VERSION=1
uint8 STOP=0
uint8 BEGIN_SESSION=1
uint8 ARM=2
uint8 VELOCITY=3
uint8 protocol_version
uint32 boot_id
uint32 session_id
uint32 sequence
# MCU monotonic deadline; STOP ignores this field. Never ROS epoch time.
uint64 valid_until_us
uint8 mode
float32[4] velocity_rad_s
