import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import (Node, PushRosNamespace)
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, ExecuteProcess, 
                            LogInfo, RegisterEventHandler, TimerAction, GroupAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


# sim:='False' perception:='zed2i' planning:='HRHCS' controller:='pure-pursuit'

def generate_launch_description():
    ns = LaunchConfiguration('ns') 

    ns_arg = DeclareLaunchArgument(
        'ns',
        # CHECK this is an appropriate default 
        default_value='agent0',
        description='this argument is used to configure the applied namespace for the nodes associated with this instance of the '
    )

    agent_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('bringup'), 'launch'),
            '/agent_base_launch.py']),
        )
    
    agent_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(ns),
            agent_base,
        ]
    )


    return LaunchDescription([
        ns_arg,
        agent_with_namespace,
        RegisterEventHandler(
            OnProcessStart(
                target_action=agent_with_namespace,
                on_start=[
                    LogInfo(msg='Agent: ' + ns + ' has started'),
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])