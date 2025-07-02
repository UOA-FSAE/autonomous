#!/bin/bash
source install/setup.bash
colcon build --packages-select cone_map_foxglove_visualiser && ros2 run cone_map_foxglove_visualiser visualiser

