import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # get cones
        launch_ros.actions.Node(
            package='simulator',
            executable='get_cones',
            name='get_cones',
        ),

        # car position
        launch_ros.actions.Node(
            package='simulator',
            executable='get_car_position',
            name='get_car_position',
        ),

        # path generation
        launch_ros.actions.Node(
            package='path_planning',
            executable='trajectory_generation',
            name='trajectory_generation',
            parameters=[{'debug': True, 
                        'timer': 1.0}],
        ),

        # path optimization
        launch_ros.actions.Node(
            package='path_planning',
            executable='trajectory_optimisation',
            name='trajectory_optimisation',
            parameters=[{'delete': False,
                        'interpolate': False}],
        ),

        # controller
        launch_ros.actions.Node(
            package='stanley_controller',
            executable='controller',
            name='controller',
        ),

        # simulator controller
        launch_ros.actions.Node(
            package='simulator',
            executable='set_car_controls',
            name='set_car_controls'
        ),

        # steer torque
        # launch_ros.actions.Node(
        #     package='steer_torque_from_ackermann',
        #     executable='steer_torque_from_ackermann',
        #     name='steer_torque_from_ackermann',
        # ),

        # path viz
        launch_ros.actions.Node(
            package='path_planning_visualization',
            executable='visualize',
            name='path_viz',
        ),

        # track viz
        launch_ros.actions.Node(
            package='cone_map_foxglove_visualizer',
            executable='visualizer',
            name='track_viz',
        ),
    ])
