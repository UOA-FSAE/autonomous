import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch_ros.actions import (Node, PushRosNamespace)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    sim = LaunchConfiguration('sim')
    vis = LaunchConfiguration('sim')
    perception = LaunchConfiguration('perception')
    synthesis = LaunchConfiguration('perception')
    planning = LaunchConfiguration('planning')
    controller = LaunchConfiguration('controller')

    ns_arg = DeclareLaunchArgument(
        'ns',
        # CHECK this is an appropriate default 
        default_value='agent0',
        description='this argument is used to configure the applied namespace for the nodes associated with this instance of the '
    )
    vis_arg = DeclareLaunchArgument(
        'vis',
        # CHECK this is an appropriate default 
        default_value='True',
        description='this argument specifies if visualization nodes are running'
    )
    use_sim_arg = DeclareLaunchArgument(
        'sim',
        # CHECK this is an appropriate default 
        default_value='False', # defaults to gokart/car configurations
        description='this argument is used to configure the applied perception algorithm'
    )
    perception_arg = DeclareLaunchArgument(
        'perception',
        # CHECK this is an appropriate default 
        default_value='zed2i_Yolov7', # Purely uses the Stereolabs Zed 2i camera
        description='this argument is used to configure the applied perception algorithm'
    )
    planning_arg = DeclareLaunchArgument(
        'planning',
        # CHECK this is an appropriate default 
        default_value='HRHCS', # 'Hierarchical Receding Horizon Centre Seeking' algorithm created by Tanish
        description='this argument is used to configure the applied planning algorithm'
    )
    controller_arg = DeclareLaunchArgument(
        'controller',
        # CHECK this is an appropriate default 
        default_value='pure_pursuit', # steering point look ahead P-controller created by Zane
        description='this argument is used to configure the applied controller algorithm'
    )
    
    base_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moa_bringup'), 'launch'),
            '/base_launch.py']),
    )
    
    planning_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moa_bringup'), 'launch'),
            planning + '_launch.py']),
        launch_arguments={'': 'carrot1'}.items(),
    )
    # sythesis nodes are 
    
    synthesis_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moa_bringup'), 'launch'),
            '/physical_sensors_launch.py']),
        launch_arguments={'': 'carrot1'}.items(),
    )
    
    control_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moa_bringup'), 'launch'),
            '/physical_sensors_launch.py']),
        launch_arguments={'': 'carrot1'}.items(),
    )
    

    launchDescriptions = [
        ns_arg,
        vis_arg,
        use_sim_arg,
        perception_arg,
        planning_arg,
        controller_arg,
        base_nodes,
        sensor_nodes,
        planning_nodes,
        synthesis_nodes,
        control_nodes,
    ]
    
    
    if (sim):
        #TODO
        sensor_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('bringup'), 'launch'),
                '/acquisition_sim_launch.py']),
        )
    else:
        #TODO
        sensor_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('bringup'), 'launch'),
                '/acquisition_phy_launch.py']),
    )
    launchDescriptions.append(sensor_nodes)
        
    #visualisation
    if (vis):
        #TODO
        launchDescriptions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('bringup'), 'launch'),
                    '/visualisation_launch.py']
                )
            )
        )
    
    return LaunchDescription(launchDescriptions)