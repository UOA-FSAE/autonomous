import launch
import launch_ros.actions
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():  
    return launch.LaunchDescription([
        DeclareLaunchArgument(
        'can_id',
        default_value='0x300',
        description='The frame ID for the CAN messages containing Ackermann commands that are sent to the car'
        ),

        DeclareLaunchArgument(
            'candapter_topic',
            default_value='pub_raw_can',
            description='The subscriber and publisher topic for the Can Adapter node'
        ),
        
        # acker to can
        launch_ros.actions.Node(
            package='moa_controllers',
            executable='ack_to_can_node',
            name='ack_to_can_node',
            parameters=[{'can_id': launch.substitutions.LaunchConfiguration('can_id')}],
        ),

        # candapter
        launch_ros.actions.Node(
            package='CanTalk',
            executable='candapter_node',
            name='candapter_node',
            remappings=[('can',launch.substitutions.LaunchConfiguration('candapter_topic'))],
        ),

        # foxglove
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port':8765}],
        ),
        
        # cone detection - aruco detection (ANY)
        launch_ros.actions.Node(
            package='aruco_detection',
            executable='aruco_detection',
            name='aurco_detection'
        ),

        # cone map
        launch_ros.actions.Node(
            package='cone_mapping',
            executable='listener',
            name='cone_map'
        ),

        # path planning algorithm - center line (ANY)
        launch_ros.actions.Node(
            package='path_planning',
            executable='center_line',
            name='center_line',
        ),

        # path planning controller - head to goal (ANY)
        launch_ros.actions.Node(
            package='head_to_goal_control',
            executable='controller',
            name='head_to_goal_controller'
        ),

        launch_ros.actions.Node(
            package='moa_controllers',
            executable='as_status_node',
            name='as_status_node',
        ),
  ])
