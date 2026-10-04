# Lecturas del Multi-ranger + ToF inferior del Flow Deck, en metros.
# Convención REP-117:
#   +inf -> sin retorno (nada dentro de range_max)
#   -inf -> objeto más cerca que range_min
#   NaN  -> lectura inválida o sensor no disponible
# header.frame_id = base_link. Posición de cada sensor: config/frames.yaml.
std_msgs/Header header
float32 front
float32 back
float32 left
float32 right
float32 up
float32 down
float32 range_min
float32 range_max
