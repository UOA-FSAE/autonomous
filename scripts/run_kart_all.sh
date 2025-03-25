#!/bin/bash
./run_zed_launch.sh
./run_path_planning.sh
./run_controller.sh

ros2 run CANTalk candapter_node



