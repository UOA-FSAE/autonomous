import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():  
    return launch.LaunchDescription([
        # launch file arguments
        DeclareLaunchArgument(
            'max_speed',
            default_value="2.0",
            description='maximum speed of the vehicle when R2 is fully pressed on the joystick'
        ),

        DeclareLaunchArgument(
            'max_angle',
            default_value="25.0",
            description='maximum angle of the vehicle when L3 is fully pressed to the right/left on the joystick'
        ),

        DeclareLaunchArgument(
            'use_ds4drv',
            default_value="True",
            description='Its a type of connection, whether to use it or not. Recommended'
        ),

        DeclareLaunchArgument(
            'verbose',
            default_value="False",
            description='Whether to print output'
        ),

        # add base launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("moa_bringup"),'/launch','/base.launch.py'
            ])
        ),

        # joystick teleoperation node
        Node(
            package='moa_controllers',
            executable='joystick_teleop',
            name='joystick_teleop',
            parameters=[{'max_speed', LaunchConfiguration("max_speed")},
                        {'max_angle', LaunchConfiguration("max_angle")},
                        {'use_ds4drv', LaunchConfiguration("use_ds4drv")},
                        {'verbose', LaunchConfiguration("verbose")}]
        ),
    ])
