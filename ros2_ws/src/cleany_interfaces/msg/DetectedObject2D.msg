# 사용자에게 표시하는 snapshot 내 번호이며 1부터 시작합니다.
uint32 object_id

# Normalized semantic object type; Gemini may correct the detector class.
string label
# Original detector class for geometric association; empty if unavailable.
string detector_label

# [0, 1]로 정규화된 검출 신뢰도입니다.
float32 confidence

# 모델이 선택적으로 제공하는 분류 의미입니다. 빈 값은 정보 없음을 뜻하며
# label로 처분 방법을 추론해도 된다는 뜻이 아닙니다. 값은 trash/lost_item/review입니다.
string sorting_category
# Retained while existing collection consumers require classification evidence.
string sorting_reason

# 설정된 대상 좌표계 원점에서 강건하게 추정한 대표점까지의 거리입니다.
# distance_valid=false이면 bbox에 유효한 정렬 깊이가 충분하지 않다는 뜻이며
# 자율 조작 대상으로 선택하면 안 됩니다.
bool distance_valid
float32 distance_m

# 정렬된 RGB 영상의 왼쪽/위쪽 최솟값과 오른쪽/아래쪽 최댓값입니다.
# 좌표는 픽셀 단위이며 소수일 수 있습니다.
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# 베이스가 고정된 추적 session에서 성공한 관측 사이에 유지되는 식별자입니다.
# TRACKED / AMBIGUOUS / INVALID / UNTRACKED 중 TRACKED만 선택할 수 있습니다.
string track_id
string tracking_state
bool position_valid
geometry_msgs/Point representative_position
# Head RGB-D estimates retained for the physical target filters. Zero means
# unavailable. Width/height are two horizontal support-plane footprint extents
# from foreground depth (instance mask when available), not image-axis bbox
# lengths or object vertical height. Selected inspection/grasp checks still
# validate the actual object and required jaw opening.
float32 median_depth_m
float32 estimated_width_m
float32 estimated_height_m
