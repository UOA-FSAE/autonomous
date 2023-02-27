import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fox_glove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('foxglove_bridge'), 'launch'),
            'foxglove_bridge_launch.xml']
        ),
        launch_arguments={'port': '4747'}.items(),
    )

    return LaunchDescription(
        fox_glove_bridge
    )
