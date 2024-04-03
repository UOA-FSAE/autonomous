import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # foxglove bridge
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port':8765}]
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

        # controller viz
        launch_ros.actions.Node(
            package='pure_pursuit_visualizer',
            executable='visualizer',
            name='controller_viz',
        ),
        
        # base TF
        launch_ros.actions.Node(
            package='cone_map_foxglove_visualizer',
            executable='base_tf',
            name='base_tf_viz',
        ),
        
        # cone map
        launch_ros.actions.Node(
            package='cone_map_foxglove_visualizer',
            executable='visualizer',
            name='cone_map_viz',
        ),
        
    ])