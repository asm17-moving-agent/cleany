# 독립적인 안전 어댑터의 근거이며 소프트웨어 초기화 명령이 아닙니다.
# 필드를 검증할 수 없는 어댑터는 안전하지 않은 값을 보고해야 합니다.
std_msgs/Header header
bool protection_ready
bool estop_engaged
bool physical_release_confirmed
# 전체 로봇의 물리적 정지 근거이며 명령 또는 취소 확인이 아닙니다.
bool stopped
string detail
