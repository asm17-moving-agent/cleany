std_msgs/Header header
# Supervisor 프로세스가 시작할 때마다 바뀝니다. 단조 시각 필드는 이 epoch에 속합니다.
string epoch
uint64 revision
string operating_state
cleany_interfaces/MissionPermission[] permissions
