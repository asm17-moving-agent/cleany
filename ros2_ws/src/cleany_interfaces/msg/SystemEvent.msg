# 구조화된 Supervisor 감사 이벤트입니다. monotonic_s는 발행 epoch에 속합니다.
std_msgs/Header header
string epoch
uint64 sequence
float64 monotonic_s
# 예: STATE_CHANGED, FAULT_RAISED, FAULT_CLEARED, FAULT_RESET_ACCEPTED, LEASE_REVOKED,
# MISSION_STOPPED, RECOVERY_RUNNING, RECOVERY_SUCCEEDED, RECOVERY_UNSUPPORTED.
string kind
string code
string source
string severity
string detail
