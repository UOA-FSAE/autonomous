#!/bin/bash
source install/setup.bash
colcon build --packages-select path_planning && ros2 run path_planning center_line
