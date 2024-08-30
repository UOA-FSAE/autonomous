#!/bin/bash
source install/setup.bash
colcon build --packages-select moa_controllers && ros2 launch moa_bringup base.py

