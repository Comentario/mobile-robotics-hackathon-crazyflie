# Mission states (README diagram). "Avoid" is not a mission state:
# local_navigator handles it without changing waypoint.
uint8 IDLE=0
uint8 PREFLIGHT=1       # Check
uint8 TAKEOFF=2
uint8 NAVIGATE=3
uint8 LANDING=4
uint8 COMPLETED=5
uint8 PAUSED=6
uint8 ABORTED=7         # Safe

std_msgs/Header header
uint8 state
uint32 waypoint_index
uint32 waypoint_count
string reason
