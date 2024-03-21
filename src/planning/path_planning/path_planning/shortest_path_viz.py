#!/usr/bin/python3
from foxglove_msgs.msg import LinePrimitive, Color, SceneEntity, SceneUpdate, ArrowPrimitive, SpherePrimitive, PoseInFrame, PosesInFrame
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3, Quaternion, PoseArray
#from moa_msgs.msg import AllTrajectories, AllStates
from moa_msgs.msg import AllTrajectories
from ackermann_msgs.msg import AckermannDrive

from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import Int16, Int32MultiArray

import rclpy
from rclpy.node import Node
import numpy as np

class pub_viz(Node):
    def __init__(self):
        super().__init__("shortest_path_viz")
        self.get_logger().info("shortest path visulisation node started")

        self.create_subscription(PoseArray, "moa/selected_trajectory", self.show_paths, 5)
        self.pubviz = self.create_publisher(SceneUpdate, 'visualization_trajectories', 10)

        self.id = 1


    def show_paths(self, msg:PoseArray):
        path = msg.poses
        points = []
        for i in range(len(path)):
            points.append(path[i].position)

            
        tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
        args = {'type': LinePrimitive.LINE_STRIP,
                'pose': Pose(position=Point(x=0.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
                'thickness': 1.0,
                'scale_invariant': True,
                'points': points,
                'color': tcols}
        chosen_line = LinePrimitive(**args)

        # scene entity encapsulates these primitive objects
        sargs = {'timestamp': Time(sec=0,nanosec=0),
                    'frame_id': 'global_frame',
                    'id': f'{self.id}',
                    'lifetime': Duration(sec=1,nanosec=0),
                    'frame_locked': False,
                    'lines': [chosen_line]}
        # scene update is a wrapper for scene entity
        scene_update_msg = SceneUpdate(entities=[SceneEntity(**sargs)])

        self.pubviz.publish(scene_update_msg)
        self.get_logger().info("Published msg")

        self.id += 1

        return



def main():
    rclpy.init()
    nde = pub_viz()
    rclpy.spin(nde)
    nde.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()