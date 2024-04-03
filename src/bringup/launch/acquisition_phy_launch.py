import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch_ros.actions import (Node, PushRosNamespace)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    camera = LaunchConfiguration('ns') 
    
    # camera
    camera_arg = DeclareLaunchArgument(
        'camera',
        # CHECK this is an appropriate default 
        default_value='zed2i', # Purely uses the Stereolabs Zed 2i camera
        description='this argument is used to declare the camera being used'
    )
    
    
    
    launch_description = [
        camera_arg
    ]
    
    if (camera == 'zed2i'):
        launch_description.append(
            Node(
            package='zed_wrapper',
            executable='ack_to_can_node',
            name='ack_to_can_node',
            parameters=[{'camera_model': 'zed2i'}]
            )
        )
    else:
        raise NotImplementedError('camera parameter is not supported')
    
    
    
    return LaunchDescription(launch_description)
