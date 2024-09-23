import launch
import launch_ros.actions
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():  
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port':8765}]
        ),
        
        launch_ros.actions.Node(
            package='cone_map_foxglove_visualizer',
            executable='visualizer',
            name='cone_map_visualizer',
        ),
        
        launch_ros.actions.Node(
            package='path_planning_visualization',
            executable='visualize',
            name='path_planning_visualizer',
        ),
        
        launch_ros.actions.Node(
            package='pure_pursuit_visualizer',
            executable='visualizer',
            name='controller_visualizer',
        ),

  ])

# TO DO: Add the visualization nodes to the launch file