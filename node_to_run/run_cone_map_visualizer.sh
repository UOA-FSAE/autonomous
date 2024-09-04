#!/bin/bash
source install/setup.bash
colcon build --packages-select cone_map_foxglove_visualizer && ros2 run cone_map_foxglove_visualizer visualizer

