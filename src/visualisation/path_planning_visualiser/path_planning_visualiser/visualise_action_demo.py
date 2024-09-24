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
        super().__init__("publish_actions")
        self.get_logger().info("publish action node started")

        self.pubviz = self.create_publisher(SceneUpdate, 'visualization_trajectories', 10)
        # sub to all trajectories points and states
        self.create_subscription(PoseArray, "moa/chosen_actions", self.show_paths, 10)

        self.id = 1

    def show_paths(self, msg: PoseArray):
        poses = msg.poses

        points = []
        for j in range(len(poses)):
            # get a particular pose
            _ = poses[j].position
            points.append(_)

        args = {'type': LinePrimitive.LINE_STRIP,
                'pose': Pose(position=Point(x=0.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
                'thickness': 2.0,
                'scale_invariant': True,
                'points': points,
                'color': Color(r=0.0,g=255.0,b=0.0,a=1.0)}
        tmp = LinePrimitive(**args)


        # scene entity encapsulates these primitive objects
        sargs = {'timestamp': Time(sec=0,nanosec=0),
                    'frame_id': 'global_frame',
                    'id': f'{self.id}',
                    'lifetime': Duration(sec=2,nanosec=100),
                    'frame_locked': False,
                    'lines': [tmp]}
        
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