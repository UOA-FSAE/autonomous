import launch
import launch_ros.actions

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

        launch_ros.actions.Node(
            package='moa_controllers',
            executable='as_status_node',
            name='as_status_node'),

        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'),
  ])