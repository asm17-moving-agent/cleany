# Independent sensor outcome for a pinned object. A controller/gripper command
# completing is not an outcome. Producers must preserve the original identity.
std_msgs/Header header
string arm
string reference_id
string wrist_reference_id
string source_snapshot_id
uint32 source_object_id
string destination_id
bool contact_confirmed
bool release_confirmed
bool placement_confirmed
