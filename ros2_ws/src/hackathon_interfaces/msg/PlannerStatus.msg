uint8 IDLE=0
uint8 OK=1
uint8 NO_PATH=2
uint8 TIMEOUT=3
uint8 NO_MAP=4

std_msgs/Header header
uint8 status
uint32 replan_count
float32 plan_time       # s
string reason
