# Snapshot-local number presented to users. Numbering starts at 1.
uint32 object_id

# Normalized semantic object type; Gemini may correct the detector class.
string label
# Original detector class for geometric association; empty if unavailable.
string detector_label

# Detector confidence normalized to [0, 1].
float32 confidence

# Optional model-provided sorting semantics. Empty means unavailable, never
# permission to infer disposal from the label. Values: trash/lost_item/review.
string sorting_category
# Retained while existing collection consumers require classification evidence.
string sorting_reason

# Robust representative-point distance from the configured target-frame
# origin. distance_valid=false means the bbox did not contain enough valid
# aligned depth and must not be selected for autonomous manipulation.
bool distance_valid
float32 distance_m

# Left/top minimum and right/bottom maximum coordinates in the aligned RGB
# image. Coordinates are expressed in pixels and may be fractional.
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Identity across successful observations in a fixed-base tracking session.
# TRACKED / AMBIGUOUS / INVALID / UNTRACKED. Only TRACKED is selectable.
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
