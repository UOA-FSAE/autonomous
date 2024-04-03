import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch_ros.actions import (Node, PushRosNamespace)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

import launch
import launch_ros.actions

def generate_launch_description():
    
    aruco_arg = DeclareLaunchArgument(
        'aruco',
        # CHECK this is an appropriate default 
        default_value='True',
        description='this argument is used to specify whether to use Aruco markers in place of cones. True for Aurco Markers. False for Cones'
    )
    
    # NOTE this is not currently used
    aruco_type = DeclareLaunchArgument(
        'aruco_type',
        # CHECK this is an appropriate default 
        default_value='6',
        description='(NOTE this is not currently used) this argument is used to specify the type of aruco markers used'
    )
    
    # NOTE this is not currently used
    stripe_arg = DeclareLaunchArgument(
        'stripe',
        # CHECK this is an appropriate default 
        default_value='True',
        description='(NOTE this is not currently used) this argument is used to indicate whether cones have stripes'
    )
    
    detection_aruco = launch_ros.actions.Node(
        package='aruco_detection',
        executable='aruco_detection',
        name='aruco_detection'
    )
         
    return launch.LaunchDescription([
        aruco_arg,
        aruco_type,
        stripe_arg,
        detection_aruco
        ])