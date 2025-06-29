import launch
import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions import Node
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
            package="moa_controllers",
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
            parameters=[{'port':8765, 'topic_whitelist': ["/moa/image_throttled", "/moa/cmd_vel", "/moa/selected_trajectory"]}]
        ),
    ])
