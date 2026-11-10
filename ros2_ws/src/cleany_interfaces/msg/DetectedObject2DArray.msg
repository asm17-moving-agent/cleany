# 전체 검출이 공유하는 촬영 시각과 RGB 광학 좌표계입니다. 깊이가 유효한 검출은
# 대상 좌표계 거리, 검출 신뢰도, 원래 검출 순서로 정렬합니다.
# 깊이가 무효인 검출은 그 뒤에 배치합니다.
std_msgs/Header header

# 후속 선택 물체 검사 요청에 사용하는 내부 해석 없는 식별자입니다.
string snapshot_id

DetectedObject2D[] detections

string tracking_session_id
string tracking_epoch
# representative_position의 좌표계이며 RGB header.frame_id와 독립적입니다.
string representative_frame
# 이전에 추적했으나 이번 관측에는 없는 물체와 마지막 snapshot 내 참조입니다.
# 제거 여부는 실행 이력을 사용해 해석합니다.
TrackedObjectReference[] missing_objects
