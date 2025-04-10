import launch
import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='moa',
            package='zed_launch',
            executable='zed_launch_node',
            name='perception'
        ),
        Node(
            namespace='moa',
            package='cone_mapping',
            executable='kalman_filter',
            name='kalman_filter'
        ),
        Node(
            namespace='moa',
            package='path_planning',
            executable='fasttube',
            name='centerline_planner'
        ),
        Node(
            namespace='moa',
            package='stanley_controller',
            executable='controller',
            name='stanley_controller'
        ),
        launch_ros.actions.Node(
            namespace='moa',
            package="CanTalk",
            executable="candapter_node",
            output="screen"
        ),
        launch_ros.actions.Node(
            namespace='moa',
            package="moa_controllers",
            executable="ack_to_can_node",
            name='ack_to_can',
            output="screen"
        )
    ])
