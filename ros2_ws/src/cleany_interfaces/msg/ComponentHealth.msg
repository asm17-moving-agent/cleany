# Semantic availability, published by the component owner; not a mere heartbeat.
uint8 UNKNOWN=0
uint8 AVAILABLE=1
uint8 UNAVAILABLE=2
std_msgs/Header header
string source
uint8 status
string detail
