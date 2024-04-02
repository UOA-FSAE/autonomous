#!/bin/bash
source install/setup.bash
colcon build --packages-select CanTalk && ros2 run CanTalk candapter_node
