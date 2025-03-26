import launch
import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_launch',
            namespace='perception_pkg',
            executable='zed_launch_node',
            name='perception'
        ),
        Node(
            package='path_planning',
            namespace='planning_pkg',
            executable='fasttube_planner',
            name='path_planner'
        ),
        Node(
            package='stanley_controller',
            namespace='controller_pkg',
            executable='controller',
            name='stanley_controller'
        ),
        launch_ros.actions.Node(
            package="CANTalk",
            executable="candapter_node",
            output="screen"
        )
    ])
