from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument for recording name
    recording_name_arg = DeclareLaunchArgument(
        'recording_name',
        default_value='default_recording',
        description="""
    This launch file is used to start the ZED camera node with or 
    without recording functionality.
    Usage:
        - Without recording: ros2 launch zed_perception.launch.py
        - With recording: ros2 launch zed_perception.launch.py recording_name:=<NAME>
    
    The 'recording_name' argument is optional. If provided, the camera node will 
    initiate a recording session using the specified file name.
    """
    )

    # Access the recording name configured by the user
    recording_name = LaunchConfiguration('recording_name')

    # Define the node, including the recording name as a command-line argument
    camera_node = Node(
        package='zed_perception',
        executable='zed_launch_node',
        name='zed_camera',
        output='screen',
        arguments=[recording_name]
    )

    return LaunchDescription([
        recording_name_arg,
        camera_node
    ])