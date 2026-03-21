from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_prefix
from launch.actions import OpaqueFunction
import os

def generate_launch_description():
    package_name = 'cone_mapping'
    package_dir = os.path.join(get_package_prefix(package_name), 'lib', package_name)   # directory of installed node names
    mappers = os.listdir(package_dir)  # retrieves installed node names 

    launch_descriptions = [        
        # launch arguments
        DeclareLaunchArgument(
            'node_name',
            default_value='kalman_filter',
            description='which node from cone_mapping package to launch'
        )
    ]

    node = OpaqueFunction(function=get_node, args=[package_name, mappers])    # get a list of actions
    launch_descriptions.append(node)

    return LaunchDescription(launch_descriptions)

def get_node(context, package_name, mappers):
    NODE = None
    node_2_run = LaunchConfiguration('node_name').perform(context)  # get runtime value of argument

    for node_name in mappers:
        if node_name == node_2_run:
            node = Node(
                package=package_name,
                executable=node_name,
                name=node_name,
            )
            NODE = node
            break
    else:
        raise Exception(f"selected node {node_2_run} does not exist!")

    return NODE
