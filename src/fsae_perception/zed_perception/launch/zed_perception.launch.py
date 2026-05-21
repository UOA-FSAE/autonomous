import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    recording_name_arg = DeclareLaunchArgument(
        'recording_name',
        default_value='default_recording',
        description='Optional SVO recording file name for offline replay.'
    )

    use_wrapper_arg = DeclareLaunchArgument(
        'use_wrapper',
        default_value='false',
        description='If true, launch the official ZED wrapper and wrapper-based perception node.'
    )

    wrapper_image_topic_arg = DeclareLaunchArgument(
        'wrapper_image_topic',
        default_value='/zed/zed_node/rgb/color/rect/image',
        description='Wrapper rectified image topic.'
    )

    wrapper_depth_topic_arg = DeclareLaunchArgument(
        'wrapper_depth_topic',
        default_value='/zed/zed_node/depth/depth_registered',
        description='Wrapper registered depth map topic (32FC1, meters).'
    )

    wrapper_camera_info_topic_arg = DeclareLaunchArgument(
        'wrapper_camera_info_topic',
        default_value='/zed/zed_node/rgb/color/rect/camera_info',
        description='Wrapper camera info topic for intrinsics.'
    )

    wrapper_odom_topic_arg = DeclareLaunchArgument(
        'wrapper_odom_topic',
        default_value='/zed/zed_node/odom',
        description='Wrapper odometry topic for car pose and velocity.'
    )

    wrapper_camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model passed to the official wrapper launch.'
    )

    recording_name = LaunchConfiguration('recording_name')
    use_wrapper = LaunchConfiguration('use_wrapper')
    wrapper_image_topic = LaunchConfiguration('wrapper_image_topic')
    wrapper_depth_topic = LaunchConfiguration('wrapper_depth_topic')
    wrapper_camera_info_topic = LaunchConfiguration('wrapper_camera_info_topic')
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

    custom_zed_config = os.path.join(
        get_package_share_directory('zed_perception'),
        'config',
        'zed_wrapper_override.yaml'
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
            'ros_params_override_path': custom_zed_config
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
            'depth_topic': wrapper_depth_topic,
            'camera_info_topic': wrapper_camera_info_topic,
            'odom_topic': wrapper_odom_topic,
            'model_name': 'cone_detection_model.engine'
        }],
        condition=IfCondition(use_wrapper)
    )

    return LaunchDescription([
        recording_name_arg,
        use_wrapper_arg,
        wrapper_image_topic_arg,
        wrapper_depth_topic_arg,
        wrapper_camera_info_topic_arg,
        wrapper_odom_topic_arg,
        wrapper_camera_model_arg,
        custom_camera_node,
        zed_wrapper_launch,
        wrapper_perception_node
    ])
