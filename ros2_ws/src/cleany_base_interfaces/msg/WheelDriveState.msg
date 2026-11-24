# Sampled with WheelState. All arrays use FL, FR, RL, RR.
uint32 boot_id
uint32 sequence
uint64 timestamp_us
uint16 pwm_period_ticks
uint32 pwm_frequency_hz
int16[4] duty_ticks
float32[4] requested_pwm_percent
# First accepted packet with a changed target, timed at the MCU callback.
uint32 first_sequence
uint64 received_us
int32[4] start_counts
# First control samples with >=1 / >=5 counts of signed progress. -1 while pending.
int32[4] first_us
int32[4] fifth_us
int16[4] onset_ticks
