import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # acceleration
	launch_ros.actions.Node(
	    package="acceleration",
	    executable="controller",
	    name="acceleration"
	),

	# cone map
        launch_ros.actions.Node(
            package='aruco_detection',
            executable='aruco_detection',
            name='aruco_detection'
        ),

        # cone mapping
        launch_ros.actions.Node(
            package='cone_mapping',
            executable='dbscan',
            name='dbscan',
        ),

        # car position
        launch_ros.actions.Node(
            package='cone_mapping',
            executable='car_position',
            name='car_position',
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
