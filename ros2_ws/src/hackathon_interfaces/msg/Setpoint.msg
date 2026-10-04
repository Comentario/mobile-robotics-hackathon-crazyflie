# Comando de navegación de alto nivel para el controlador onboard del Crazyflie.
# local_navigator publica /hk/cmd/nav; solo safety_supervisor publica /hk/cmd/safe,
# que es lo único que escuchan los adapters.
uint8 MODE_POSITION=0   # ir a position con yaw
uint8 MODE_VELOCITY=1   # seguir velocity (frame map) manteniendo position.z
uint8 MODE_HOVER=2      # mantener la posición actual

std_msgs/Header header  # frame_id = map
uint8 mode
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 yaw             # rad, frame map
