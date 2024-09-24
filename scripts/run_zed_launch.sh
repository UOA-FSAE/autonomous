#!/bin/bash
source install/setup.bash
colcon build --packages-select zed_launch && ros2 run zed_launch zed_launch_node recording2.svo2