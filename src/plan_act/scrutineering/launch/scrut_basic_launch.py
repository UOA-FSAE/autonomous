import launch
import launch_ros.actions

def generate_launch_description():  
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='scrutineering',
            executable='scrut_mission_node',
            name='scrut_mission_node'),
  ])