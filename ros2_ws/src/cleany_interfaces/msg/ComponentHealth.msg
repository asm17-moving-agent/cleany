# 컴포넌트 소유자가 발행하는 의미상 사용 가능 상태이며 단순 heartbeat가 아닙니다.
uint8 UNKNOWN=0
uint8 AVAILABLE=1
uint8 UNAVAILABLE=2
std_msgs/Header header
string source
uint8 status
string detail
