import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # get urdf file path
    path = os.path.join(get_package_share_directory('moa_description'))
    xacro_file = os.path.join(path,'urdf','robot.urdf.xacro')
    # process xacro file
    robot_description = xacro.process_file(xacro_file)

    # robot state publisher node
    robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description.toxml()
            }]
        )
    
    # joint state publisher node
    # joint_state_pub_node = Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         remappings=[('joint_states','/joint_states')]
    #     )

    return LaunchDescription([robot_state_pub_node])