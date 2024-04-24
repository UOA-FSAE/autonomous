import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # cone detect
        launch_ros.actions.Node(
            package='aruco_detection',
            executable='aruco_detection',
            name='aruco_detection'
        ), 

        # cone mapping
        launch_ros.actions.Node(
            package='cone_mapping',
            executable='dbscan',
            name='listener',
        ),

        # path generation
        launch_ros.actions.Node(
            package='path_planning',
            executable='trajectory_generation',
            name='trajectory_generation',
            parameters=[{'debug': True, 
                         'timer': 0.5}],
        ),

        # path optimization
        launch_ros.actions.Node(
            package='path_planning',
            executable='trajectory_optimisation',
            name='trajectory_optimisation',
            parameters=[{'debug': True}],
        ),

        # controller
        launch_ros.actions.Node(
            package='moa_controllers',
            executable='trajectory_follower',
            name='trajectory_follower',
        ),

        # path viz
        launch_ros.actions.Node(
            package='path_planning_visualization',
            executable='visualize2',
            name='path_viz',
        ),

        # track viz
        launch_ros.actions.Node(
            package='cone_map_foxglove_visualizer',
            executable='visualizer',
            name='track_viz',
        ),

        # launch_ros.actions.Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        #     name='foxglove_bridge',
        #     parameters=[{'port':8765}]
        # ),
    ])