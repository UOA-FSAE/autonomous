#!/bin/bash
source install/setup.bash
colcon build --packages-select head_to_goal_control && ros2 run head_to_goal_control controller
#colcon build --packages-select stanley_controller && ros2 run stanley_controller controller

