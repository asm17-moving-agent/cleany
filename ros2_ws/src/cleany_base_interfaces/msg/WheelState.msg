# Fixed-size micro-ROS contract v2. Logical signs, wheel order FL, FR, RL, RR.
uint8 PROTOCOL_VERSION=2
uint8 STOPPED=0
uint8 VELOCITY=1
uint8 REVERSE_WAIT=2
uint8 TIMEOUT_STOP=4
uint32 FAULT_COMMAND_TIMEOUT=1
uint8 protocol_version
# Reboot/encoder baseline detection only; not a command authorization token.
uint32 boot_id
uint32 sequence
uint32 last_command_sequence
# MCU monotonic time, not ROS epoch. Adapter stamps JointState at receipt.
uint64 timestamp_us
uint32 counts_per_revolution
float32 max_velocity_rad_s
# Maximum permitted command lifetime; expiry uses valid_until_us only.
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
