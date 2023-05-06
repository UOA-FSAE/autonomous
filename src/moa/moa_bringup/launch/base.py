import launch
import launch_ros.actions

def generate_launch_description():  
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='moa_controllers',
            executable='ack_to_can_node',
            name='ack_to_can_node'),
  
        launch_ros.actions.Node(
            package='moa_driver',
            executable='can_interface_jnano',
            name='can_interface_jnano'),
  ])