from launch import LaunchDescription
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():  
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch'),
                        '/zed2i.launch.py'])
    )

    aruco = launch_ros.actions.Node(
            package='test_plan_act_algorithems',
            executable='aruco_triangle',
            name='aruco_triangle')
        
    return LaunchDescription([
        zed_launch,
        aruco
    ])