"""
Launch file that sets up the complete TF tree for the trolly with sensors.
This includes:
1. Robot state publisher (publishes trolly URDF transforms)
2. Static transforms for ZED camera optical frame
3. Map to base_link transform (for visualization in RViz)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = FileContent(
        PathJoinSubstitution([FindPackageShare('trolly_description'), 'trolly_description.urdf.xml']))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Robot State Publisher - publishes TF tree from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
            arguments=[urdf]),
        
        # Static transform: map -> base_link (for RViz visualization)
        # In production, your SLAM system should publish this transform
        # For now, this keeps the robot at the origin of the map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link',
            arguments=[
                '0', '0', '0',           # x, y, z
                '0', '0', '0', '1',      # qx, qy, qz, qw
                'map', 'base_link'
            ]),
        
        # Static transform: base_link -> zed_camera_optical_frame
        # (in case ZED node doesn't publish frame_id correctly)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='zed_static_transform',
            arguments=[
                '0.25', '0', '0.95',                    # x, y, z (camera position)
                '0', '0', '0', '1',                     # qx, qy, qz, qw (no rotation)
                'base_link', 'zed_camera_optical_frame'
            ]),
    ])
