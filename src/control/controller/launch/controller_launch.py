from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
 
    controller_type = LaunchConfiguration('controller_type', default='stanley')


    return LaunchDescription([
        DeclareLaunchArgument('controller_type', default_value='stanley', description='Controller type to launch'),

        # Conditional node launching based on the parameter
        Node(
            package='controller', 
            executable=controller_type,  #reference the controller name passed as argument
            name=controller_type,  # Node name matches the controller name
            output='screen',
            parameters=[{'controller_type': controller_type}]
        ),
    ])
