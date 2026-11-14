# WheelState v2와 독립적인 고정 크기 MPU6050 통신 v2입니다.
# 보정은 Agent나 ROS 노드와 무관하게 MCU 부팅 시 한 번 실행합니다.
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
# MCU 식별자로 보정을 장치에 연결하며 교체 가능한 센서 칩의 식별자는 아닙니다.
uint8[6] device_mac
uint32 boot_id
uint32 sequence
# ROS epoch가 아닌 MCU 단조 증가 시계의 측정 시각입니다.
uint64 timestamp_us
uint8 who_am_i
uint32 fault_bits
float64[3] angular_velocity_rad_s
float64[3] acceleration_mps2
float64 temperature_c
uint8 calibration_state
uint8 calibration_reason
# 위 원시값에는 편향 보정을 적용하지 않으며 아래 각속도는 MCU가 보정한 값입니다.
# 보정 각속도와 계수는 RUNNING 또는 FALLBACK 상태에서만 사용할 수 있습니다.
float64[3] corrected_angular_velocity_rad_s
float64[3] gyro_bias_rad_s
float64[9] gyro_covariance_rad2_s2
