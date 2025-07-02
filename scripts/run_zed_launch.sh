#!/bin/bash
source install/setup.bash
colcon build --packages-select zed_launch && ros2 run zed_launch zed_launch_node
## with sim
# colcon build --packages-select zed_launch && ros2 run zed_launch zed_launch_node recording1.svo2
# colcon build --packages-select zed_launch && ros2 run zed_launch zed_launch_node recording2.svo2

ros2 run zed_launch zed_launch_node
ros2 launch foxglove_bridge foxglove_bridge_launch.xml