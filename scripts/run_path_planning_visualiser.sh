#!/bin/bash
source install/setup.bash
colcon build --packages-select path_planning_visualiser && ros2 run path_planning_visualiser visualiser



