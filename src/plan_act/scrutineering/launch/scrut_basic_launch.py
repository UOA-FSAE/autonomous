import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():  
    config = os.path.join(
        get_package_share_directory('scrutineering'),
        'config',
        'test_params.yaml'
    )
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='scrutineering',
            executable='scrut_mission_node',
            name='scrut_mission_node',
            parameters=[config]
        )
    ])