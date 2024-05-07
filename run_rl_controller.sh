#!/bin/bash
source install/setup.bash
colcon build --packages-select rl_controller && ros2 run rl_controller controller