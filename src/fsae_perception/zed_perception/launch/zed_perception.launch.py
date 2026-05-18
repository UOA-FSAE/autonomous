import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
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

    use_wrapper_arg = DeclareLaunchArgument(
        'use_wrapper',
        default_value='false',
        description='If true, launch the official ZED wrapper node and the wrapper-based perception node instead of the custom direct ZED node.'
    )

    wrapper_image_topic_arg = DeclareLaunchArgument(
        'wrapper_image_topic',
        default_value='/zed/zed_node/rgb/color/raw/image',
        description='The wrapper image topic to subscribe to for wrapper-based perception.'
    )

    wrapper_pointcloud_topic_arg = DeclareLaunchArgument(
        'wrapper_pointcloud_topic',
        default_value='/zed/zed_node/point_cloud/cloud_registered',
        description='The wrapper point cloud topic to subscribe to for wrapper-based perception.'
    )

    wrapper_odom_topic_arg = DeclareLaunchArgument(
        'wrapper_odom_topic',
        default_value='/zed/zed_node/odom',
        description='The wrapper odometry topic to subscribe to for car pose and velocity if available.'
    )

    wrapper_camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='The ZED camera model passed to the official wrapper launch file.'
    )

    recording_name = LaunchConfiguration('recording_name')
    use_wrapper = LaunchConfiguration('use_wrapper')
    wrapper_image_topic = LaunchConfiguration('wrapper_image_topic')
    wrapper_pointcloud_topic = LaunchConfiguration('wrapper_pointcloud_topic')
    wrapper_odom_topic = LaunchConfiguration('wrapper_odom_topic')
    wrapper_camera_model = LaunchConfiguration('camera_model')

    custom_camera_node = Node(
        package='zed_perception',
        executable='zed_launch_node',
        name='zed_camera',
        output='screen',
        arguments=[recording_name],
        condition=UnlessCondition(use_wrapper)
    )

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': wrapper_camera_model,
            'camera_name': 'zed',
            'node_name': 'zed_node',
            'namespace': '',
            'container_name': '',
            'ros_params_override_path': ''
        }.items(),
        condition=IfCondition(use_wrapper)
    )

    wrapper_perception_node = Node(
        package='zed_perception',
        executable='wrapper_perception_node',
        name='zed_wrapper_perception',
        output='screen',
        parameters=[{
            'image_topic': wrapper_image_topic,
            'pointcloud_topic': wrapper_pointcloud_topic,
            'odom_topic': wrapper_odom_topic,
            'model_name': 'cone_detection_model.engine'
        }],
        condition=IfCondition(use_wrapper)
    )

    return LaunchDescription([
        recording_name_arg,
        use_wrapper_arg,
        wrapper_image_topic_arg,
        wrapper_pointcloud_topic_arg,
        wrapper_odom_topic_arg,
        wrapper_camera_model_arg,
        custom_camera_node,
        zed_wrapper_launch,
        wrapper_perception_node
    ])
