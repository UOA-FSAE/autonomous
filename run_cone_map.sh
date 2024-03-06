#!/bin/bash
source install/setup.bash
colcon build --packages-select cone_mapping && ros2 run cone_mapping listener
