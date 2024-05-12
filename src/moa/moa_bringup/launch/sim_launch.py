import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # cone map
        launch_ros.actions.Node(
            package='cone_mapping',
            executable='dbscan',
            name='dbscan',
        ),

        # path optimization
        launch_ros.actions.Node(
            package='path_planning',
            executable='shortest_path',
            name='shortest_path',
        ),

        # path viz
        launch_ros.actions.Node(
            package='path_planning',
            executable='shortest_path_viz',
            name='path_viz',
        ),

        # track viz
        launch_ros.actions.Node(
            package='cone_map_foxglove_visualizer',
            executable='visualizer',
            name='track_viz',
        ),
    ])