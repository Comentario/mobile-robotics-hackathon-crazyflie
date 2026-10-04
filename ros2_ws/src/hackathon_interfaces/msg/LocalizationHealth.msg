uint8 GOOD=0
uint8 DEGRADED=1
uint8 LOST=2

std_msgs/Header header
uint8 status
float32 pose_age        # s desde la última pose recibida
string reason           # '', 'stale', 'jump', 'nan', 'overspeed'
