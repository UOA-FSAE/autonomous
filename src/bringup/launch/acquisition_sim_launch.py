import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        # convert steering encoding
        launch_ros.actions.Node(
            package='steer_torque_from_ackermann',
            executable='steer_torque_from_ackermann',
            name='steer_torque',
        ),
        
        # localization in sim
        launch_ros.actions.Node(
            package='localization',
            executable='localization',
            name='localization',
        )
    ])
