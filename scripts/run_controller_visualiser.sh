#!/bin/bash
source install/setup.bash
colcon build --packages-select pure_pursuit_visualiser && ros2 run pure_pursuit_visualiser visualiser


