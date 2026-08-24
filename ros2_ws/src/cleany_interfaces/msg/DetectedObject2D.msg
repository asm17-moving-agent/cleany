# Snapshot-local number presented to users. Numbering starts at 1.
uint32 object_id

string label

# Detector confidence normalized to [0, 1].
float32 confidence

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
