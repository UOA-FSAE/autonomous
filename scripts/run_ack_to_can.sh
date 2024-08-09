#!/bin/bash
source install/setup.bash
colcon build --packages-select moa_controllers && ros2 run moa_controllers ack_to_can_node --ros-args -p can_id:=0x300

