uint32 id          0   # ID value of cone (optional, defaults to 0 if not given)

float32 confidence 0.0 # The confidence that this is the cone, a percentage value from 0 to 100 (defaults to 0.0)

uint8 colour           # The color of the cone: 0 for Blue, 1 for Orange, 2 for Yellow, and 3 for Other

geometry_msgs/PoseWithCovariance pose # Position of cone (x, y, z) and rotation

float32 radius      0.0 # Size of cone, radius (defaults to 0.0)
float32 height      0.0 # Size of cone, height (defaults to 0.0)
