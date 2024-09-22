#!/bin/bash

gnome-terminal -- bash -c "source scripts/run_foxglove_ros.sh; exec bash"

gnome-terminal -- bash -c "source scripts/run_cone_map.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_cone_map_visualizer.sh; exec bash"

gnome-terminal -- bash -c "source scripts/run_path_planning_visualizer.sh; exec bash"
gnome-terminal -- bash -c "source scripts/run_path_planning.sh; exec bash"

gnome-terminal -- bash -c "source scripts/run_zed_launch.sh; exec bash"

