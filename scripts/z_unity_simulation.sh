#!/bin/bash
ros2 topic pub --once /race_controller/create std_msgs/msg/String "data: 'test'"

gnome-terminal -- bash -c "source scripts/run_cone_map.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_cone_map_visualizer.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_controller_visualizer.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_controller.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_foxglove_ros.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_localization.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_path_planning_visualizer.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_path_planning.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_simulation_car_control.sh; exec bash"

