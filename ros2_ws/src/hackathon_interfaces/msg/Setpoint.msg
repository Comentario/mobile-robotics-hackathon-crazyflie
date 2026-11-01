# High-level navigation command for the onboard Crazyflie controller.
# local_navigator publishes /hk/cmd/nav; only safety_supervisor publishes /hk/cmd/safe,
# which is the only thing the adapters listen to.
uint8 MODE_POSITION=0   # go to position with yaw
uint8 MODE_VELOCITY=1   # follow velocity (map frame) keeping position.z
uint8 MODE_HOVER=2      # hold the current position

std_msgs/Header header  # frame_id = map
uint8 mode
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 yaw             # rad, frame map
