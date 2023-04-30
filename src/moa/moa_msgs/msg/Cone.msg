# ID value of cone
# optional value and defults to 0 if no id is given
uint32                              id          0

# The confidence that this is the cone.
# This value is only really usfull as a value given by the
# cone dection system.
# This value is a % from 0% -> 100% 
# defult value 0.0%
float32                             confidence  0.0

# What colour is the cone
# 0: Blue
# 1: Orange
# 2: Yellow
# 3: Other
uint8                               colour     

# Position of cone
# pose is the x, y, z postion of the cone from
# given refernce point alonge with its rotation
geometry_msgs/PoseWithCovariance    pose

# Size of cone
# to get the edge of the cone you get the pose of the cone
# and add the radius of the cone to find the area the cone
# occupes.
# same goes for height
# ONLY CHANGE VALUE IF NEEDED OTHERWISE USE DEFULT
float32                             radius      0 # TODO: CHANGE THIS AS SOON AS POSIBLE
float32                             height      0
