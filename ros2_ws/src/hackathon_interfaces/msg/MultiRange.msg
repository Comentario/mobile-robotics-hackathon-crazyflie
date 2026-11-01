# Multi-ranger + Flow Deck bottom ToF readings, in metres.
# REP-117 convention:
#   +inf -> no return (nothing within range_max)
#   -inf -> object closer than range_min
#   NaN  -> invalid reading or sensor not available
# header.frame_id = base_link. Position of every sensor: config/frames.yaml.
std_msgs/Header header
float32 front
float32 back
float32 left
float32 right
float32 up
float32 down
float32 range_min
float32 range_max
