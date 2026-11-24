string mission_id
string lease_id
uint64 version
string[] capabilities
bool granted
# snapshot 발행 시점의 남은 유효 시간입니다. client는 자체 단조 시계를 사용합니다.
float64 valid_for_s
string reason
# Confirmed completion or stop, including after a lease is revoked.
bool stop_confirmed
