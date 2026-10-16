# Evidence from an independent safety adapter. No software reset command.
std_msgs/Header header
bool protection_ready
bool estop_engaged
bool physical_release_confirmed
# Whole-robot physical stop evidence; not a command/cancel acknowledgement.
bool stopped
