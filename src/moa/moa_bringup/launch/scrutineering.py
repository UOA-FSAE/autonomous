from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():  
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moa_bringup'), 'launch'),
                        '/base.py'])
    )
    
    scrut_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('scrutineering'), 'launch'),
                        '/inspection_basic_launch.py'])
    )
        
    return LaunchDescription([
        base_launch, 
        scrut_launch
    ])