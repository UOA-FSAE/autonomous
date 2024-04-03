#!/bin/bash
source install/setup.bash
#colcon build --packages-select cone_mapping && ros2 run cone_mapping baseline
colcon build --packages-select cone_mapping && ros2 run cone_mapping min_distance
#colcon build --packages-select cone_mapping && ros2 run cone_mapping dbscan
