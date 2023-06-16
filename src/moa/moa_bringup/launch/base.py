import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():  
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='moa_controllers',
            executable='ack_to_can_node',
            name='ack_to_can_node'),
        
        # # uncomment when CAN interface is completed
        # launch_ros.actions.Node(
        #     package='moa_driver',
        #     executable='can_interface_jnano',
        #     name='can_interface_jnano'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('moa_description'), 'launch'),
                            '/urdf_model.py'])),

        launch_ros.actions.Node(
            package='moa_controllers',
            executable='as_status_node',
            name='as_status_node'),

        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'),
  ])