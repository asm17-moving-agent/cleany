# Fixed-size MPU6050 transport v2, independent of WheelState v2.
# Calibration runs once on MCU boot, independently of Agent / ROS nodes.
uint8 PROTOCOL_VERSION=2
uint32 FAULT_SENSOR=1
uint32 FAULT_SATURATED=2
uint8 CALIBRATING=0
uint8 RUNNING=1
uint8 FALLBACK=2
uint8 REASON_NONE=0
uint8 REASON_TIMEOUT=1
uint8 REASON_SENSOR=2
uint8 REASON_UNSTABLE=3
uint8 REASON_SAMPLE=4
uint8 protocol_version
# MCU identity binds calibration to a device, not to a replaceable sensor chip.
uint8[6] device_mac
uint32 boot_id
uint32 sequence
# MCU monotonic acquisition time, NOT ROS epoch.
uint64 timestamp_us
uint8 who_am_i
uint32 fault_bits
float64[3] angular_velocity_rad_s
float64[3] acceleration_mps2
float64 temperature_c
uint8 calibration_state
uint8 calibration_reason
# Raw values above are never bias-corrected. Below is MCU-corrected angular rate.
# Corrected rate / coefficients are usable only in RUNNING or FALLBACK.
float64[3] corrected_angular_velocity_rad_s
float64[3] gyro_bias_rad_s
float64[9] gyro_covariance_rad2_s2
