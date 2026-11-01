uint8 NORMAL=0
uint8 SLOW=1
uint8 HOVER=2
uint8 LAND=3
uint8 ABORT=4

std_msgs/Header header
uint8 state
float32 speed_scale     # 1.0 in NORMAL, <1 in SLOW, 0 in HOVER/LAND/ABORT
string reason
