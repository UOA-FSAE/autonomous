#!/bin/bash
source install/setup.bash
colcon build --packages-select CanTalk && ros2 run cantalk candapter_node
