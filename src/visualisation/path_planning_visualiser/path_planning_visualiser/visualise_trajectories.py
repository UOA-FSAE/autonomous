#!/usr/bin/python3
from foxglove_msgs.msg import LinePrimitive, Color, SceneEntity, SceneUpdate, ArrowPrimitive, SpherePrimitive, PoseInFrame, PosesInFrame
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3, Quaternion, PoseArray
#from fsae_interfaces.msg import AllTrajectories, AllStates
from visualization_msgs.msg import Marker, MarkerArray
from fsae_interfaces.msg import AllTrajectories
from ackermann_msgs.msg import AckermannDrive

from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import Int16, Int32MultiArray,ColorRGBA

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

        # center line publisher
        self.centerline_pub = self.create_publisher(MarkerArray, "visualization_centerline", 10)

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

            for i in range(len(paths)-1):
                # choose color
                # chosen
                if i == self.chosen_trajectory:
                    # green
                    tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
                    thickness = 5.0
                # center line
                elif i == len(paths) - 1:
                    # blue
                    # tcols = Color(r=0.0, g=0.0, b=255.0, a=1.0)
                    # thickness = 3.0
                    # self.get_logger().info(f"center pts: {len(pths[i].poses)}")
                    break
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
                        'lifetime': Duration(sec=0,nanosec=500000000),
                        'frame_locked': False,
                        'lines': line_list}
            
            # show centerline
            centerline_markers = []
            centerline_markers.append(self.delete_all_markers())
            idt = 0
            for pose in paths[-1].poses:
                centerline_markers.append(self.get_marker_from_pose(idt, pose))
                idt += 1
            
            # scene update is a wrapper for scene entity
            self.pubviz.publish(SceneUpdate(entities=[SceneEntity(**sargs)]))
            self.centerline_pub.publish(MarkerArray(markers=centerline_markers))
            self.get_logger().info("Published msg")

            self.id += 1
            return
                
        self.get_logger().info("attributes not initialized")
        return


    def get_marker_from_pose(self, id, pose):
        marker = Marker()
        marker.header.frame_id = "global_frame"  # Adjust the frame ID as needed
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "global_frame"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = Vector3(x=0.3,y=0.3,z=0.3)
        marker.color = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0)
        marker.lifetime.sec = 0

        return marker
    
    def delete_all_markers(self):
        marker = Marker()
        marker.header.frame_id = "global_frame"  # Adjust the frame ID as needed
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "global_frame"
        marker.id = -1
        marker.type = Marker.CUBE
        marker.action = Marker.DELETEALL

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale = Vector3(x=0.3, y=0.3, z=0.3) 

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (opacity)

        marker.lifetime.sec = 0

        return marker

def main():
    rclpy.init()
    nde = pub_viz()
    rclpy.spin(nde)
    nde.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()