#!/bin/bash
source install/setup.bash
colcon build --packages-select head_to_goal_control && ros2 run head_to_goal_control controller


