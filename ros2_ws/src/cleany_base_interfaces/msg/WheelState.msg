# 고정 크기 micro-ROS 계약 v2입니다. 논리적 부호와 FL, FR, RL, RR 순서를 사용합니다.
uint8 PROTOCOL_VERSION=2
uint8 STOPPED=0
uint8 VELOCITY=1
uint8 REVERSE_WAIT=2
uint8 TIMEOUT_STOP=4
uint32 FAULT_COMMAND_TIMEOUT=1
uint8 protocol_version
# 재부팅과 encoder 기준점 변경 감지용이며 명령 허가 토큰이 아닙니다.
uint32 boot_id
uint32 sequence
uint32 last_command_sequence
# MCU 단조 증가 시각이며 ROS epoch가 아닙니다. 어댑터는 수신 시각으로 JointState를 표시합니다.
uint64 timestamp_us
uint32 counts_per_revolution
float32 max_velocity_rad_s
# 허용하는 명령 수명의 최댓값입니다. 실제 만료는 valid_until_us만 사용합니다.
uint16 watchdog_ms
uint16 command_age_ms
uint8 control_mode
bool enabled
uint32 fault_bits
int32[4] encoder_counts
float32[4] velocity_rad_s
float32[4] target_rad_s
float32[4] commanded_rad_s
int8[4] pwm_percent
