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
        super().__init__("publish_path_planning_msgs")
        self.get_logger().info("path planning visulisation node started")

        self.next_destination_vis = []

        self.pubviz = self.create_publisher(SceneUpdate, 'visualization_trajectories', 10)
        # sub to all trajectories points and states
        self.create_subscription(AllTrajectories, "moa/inbound_trajectories", self.set_inbound_trajectories, 10)
        self.create_subscription(AllTrajectories, "moa/trajectories", self.show_paths, 10)
        self.create_subscription(Int16, "moa/best_trajectory_index", self.get_chosen_trajectory, 10)

        self.id = 1


    def get_chosen_trajectory(self, msg: Int16) -> None:
        print(f"chosen idx got={msg.data}")
        self.chosen_trajectory = msg.data

    def set_inbound_trajectories(self, msg: AllTrajectories) -> None:
        self.inbounds = msg

    # def set_out_of_bounds_indicies(self, msg: Int32MultiArray) -> None:
    #     self.invalid_bounds_indicies = msg.data

    def show_paths(self, msg: AllTrajectories):
        if hasattr(self,"chosen_trajectory") and hasattr(self, "inbounds"):
            line_list = []
            paths = msg.trajectories
            paths.append(self.inbounds.trajectories[-1]) # append center line

            for i in range(len(paths)):
                # choose color
                # chosen
                if i == self.chosen_trajectory:
                    # green
                    tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
                    thickness = 5.0
                # center line
                elif i == len(paths) - 1:
                    # blue
                    tcols = Color(r=0.0, g=0.0, b=255.0, a=1.0)
                    thickness = 3.0
                    # self.get_logger().info(f"center pts: {len(pths[i].poses)}")
                # other lines
                else:
                    tcols = Color(r=255.0, g=255.0, b=255.0, a=0.8)
                    thickness = 1.0

                # get points
                points = []
                for j in range(len(paths[i].poses)):
                    points.append(paths[i].poses[j].position)
                args = {'type': LinePrimitive.LINE_STRIP,
                        'pose': Pose(position=Point(x=0.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
                        'thickness': thickness,
                        'scale_invariant': True,
                        'points': points,
                        'color': tcols}
                line_list.append(LinePrimitive(**args))


            # scene entity encapsulates these primitive objects
            sargs = {'timestamp': Time(sec=0,nanosec=0),
                        'frame_id': 'global_frame',
                        'id': f'{self.id}',
                        'lifetime': Duration(sec=2,nanosec=100),
                        'frame_locked': False,
                        'lines': line_list}
            
            # scene update is a wrapper for scene entity
            self.pubviz.publish(SceneUpdate(entities=[SceneEntity(**sargs)]))
            self.get_logger().info("Published msg")

            self.id += 1
            return
                
        self.get_logger().info("attributes not initialized")
        return


def main():
    rclpy.init()
    nde = pub_viz()
    rclpy.spin(nde)
    nde.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()