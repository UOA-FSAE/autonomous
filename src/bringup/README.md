This package stores ready to use launch files with default parameters for
commonly used scenarios.


ros2 launch bringup main_launch.py sim:='False' perception:='zed2i' planning:='HRHCS' controller:='pure-pursuit'

ros2 launch bringup main_gokart_launch.py perception:='zed2i' planning:='HRHCS' controller:='pure-pursuit'
ros2 launch bringup main_sim_launch.py perception:='zed2i' planning:='HRHCS' \
    controller:='pure-pursuit' ns:='agent1' 