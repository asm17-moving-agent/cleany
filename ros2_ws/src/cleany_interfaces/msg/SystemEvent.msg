# Structured Supervisor audit event. monotonic_s belongs to the emitting epoch.
std_msgs/Header header
string epoch
uint64 sequence
float64 monotonic_s
# e.g. STATE_CHANGED, FAULT_RAISED, FAULT_CLEARED, FAULT_RESET_ACCEPTED, LEASE_REVOKED,
# MISSION_STOPPED, RECOVERY_RUNNING, RECOVERY_SUCCEEDED, RECOVERY_UNSUPPORTED.
string kind
string code
string source
string severity
string detail
