import os

from ament_index_python import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

import xacro

def generate_launch_description():

    pkg_install_path = get_package_share_directory('moa_description')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_install_path
    else:
        model_path =  pkg_install_path
    
    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # get urdf file path
    path = os.path.join(get_package_share_directory('moa_description'))
    xacro_file = os.path.join(path,'urdf','moa_robot.urdf.xacro')
    # process xacro file
    robot_description = xacro.process_file(xacro_file)
    
    # robot state publisher node
    robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':robot_description.toxml(),
                         'use_sim_time':True}]
        )
    
    # joint state publisher node
    # joint_state_pub_node = Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         remappings=[('joint_states','/joint_states')],
    #         parameters=[{'robot_description':str(robot_description.toxml())}]
    #     )

    # start gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch'), '/gazebo.launch.py']),
        )
    
    # spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','my_bot',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0'],
        output='screen'
    )

    return LaunchDescription([robot_state_pub_node, gazebo, spawn_entity, gazebo_env])