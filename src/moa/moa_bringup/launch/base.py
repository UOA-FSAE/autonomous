import launch
import launch_ros.actions
from launch.actions.declare_launch_argument import DeclareLaunchArgument

def generate_launch_description():  
    return launch.LaunchDescription([
        DeclareLaunchArgument(
        'can_id',
        default_value='300',
        description='The frame ID for the CAN messages containing Ackermann commands that are sent to the car'
        ),
        
        launch_ros.actions.Node(
            package='moa_controllers',
            executable='ack_to_can_node',
            name='ack_to_can_node',
            parameters=[{'can_id': launch.substitutions.LaunchConfiguration('can_id')}],
        ),
        
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
            name='as_status_node',
        ),

        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port':8765}]),
  ])
