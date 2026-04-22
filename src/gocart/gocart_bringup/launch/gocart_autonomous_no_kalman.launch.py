import launch
import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
        'can_id',
        default_value='0x300',
        description='The frame ID for the CAN messages containing Ackermann commands that are sent to the car'
        ),
        Node(
            namespace='moa',
            package='zed_perception',
            executable='zed_launch_node',
            name='perception',
            remappings=[
                ('zed/car_position', 'car_position'),
                ('zed/cone_detection', 'cone_detection'),
            ],
        ),
        Node(
            namespace='moa',
            package='fsae_planning',
            executable='fasttube_without_kalman',
            name='centerline_planner'
        ),
        Node(
            namespace='moa',
            package='fsae_control',
            executable='controller',
            name='stanley_controller',
            parameters=[
                {'vel': 10.0},
            ],
        ),
        launch_ros.actions.Node(
            namespace='moa',
            package="CanTalk",
            executable="candapter_node",
            output="screen"
        ),
        launch_ros.actions.Node(
            namespace='moa',
            package="gocart_control",
            executable="ack_to_can_node",
            name='ack_to_can',
            parameters=[{'can_id': LaunchConfiguration('can_id')}],
            output="screen"
        ),
        # visualisation
        launch_ros.actions.Node(
            namespace='moa',
            package="path_planning_visualiser",
            executable="image_throttler",
            name='image_throttler',
            output="screen",
        ),
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port':8765, 'topic_whitelist': ["/moa/image_throttled", "/moa/cmd_vel", "/moa/selected_trajectory", "/moa/car_position", "/moa/cone_detection", "/moa/times_modified"]}]
        ),
    ])
