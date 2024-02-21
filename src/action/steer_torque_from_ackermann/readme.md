This package is for autonomous simulation car to control the car speed and steering angle

To run the node:
ros2 run steer_torque_from_ackermann talker --ros-args -p car_name:="{car_name}"

example:
ros2 run steer_torque_from_ackermann talker --ros-args -p car_name:="t"
