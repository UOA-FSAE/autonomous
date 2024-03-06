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
        self.all_paths = self.create_subscription(AllTrajectories, "moa/inbound_trajectories", self.set_inbound_trajectories, 10)
        self.create_subscription(AllTrajectories, "moa/trajectories", self.show_paths, 10)
        #self.all_states = self.create_subscription(AllStates, "moa/inbound_states", self.get_all_states, 5)
        # selected path
        #self.chosen_states = self.create_subscription(AckermannDrive, "moa/selected_trajectory", self.get_chosen_state_idx, 5)
        # self.create_subscription(PoseArray, "moa/selected_trajectory", self.get_chosen_trajectory, 5)
        # self.chosen_path = self.create_subscription(PoseArray, "moa/selected_trajectory", self.show_chosen_paths, 5)
        # self.next_destination = self.create_subscription(Pose, "moa/next_destination", self.save_next_destination, 5)
        self.create_subscription(Int16, "moa/best_trajectory_index", self.get_chosen_trajectory, 10)
        self.create_subscription(Int32MultiArray,"moa/out_of_bounds",self.set_out_of_bounds_indicies, 10)

        self.id = 1

    # def get_chosen_state_idx(self, msg:AckermannDrive) -> None: 
    #     if hasattr(self, "states"): 
    #         self.chosen_idx = np.where(np.isclose(self.states, msg.steering_angle, 1e-3))[0][0]

    def get_chosen_trajectory(self, msg: Int16) -> None:
        print(f"chosen idx got={msg.data}")
        self.chosen_trajectory = msg.data

    def set_inbound_trajectories(self, msg: AllTrajectories) -> None:
        self.inbounds = msg

    def set_out_of_bounds_indicies(self, msg: Int32MultiArray) -> None:
        self.invalid_bounds_indicies = msg.data

    def show_paths(self, msg: AllTrajectories):
        if hasattr(self,"chosen_trajectory") and hasattr(self,"invalid_bounds_indicies") and hasattr(self,'inbounds'):
            line_list = []
            pths = msg.trajectories
            pths.append(self.inbounds.trajectories[-1])
            appeneded = 0

            for i in range(len(pths)):
                # chosen
                try:
                    if i == self.chosen_trajectory + len([i for i in self.invalid_bounds_indicies if i <= self.chosen_trajectory]):
                        remove = len([i for i in self.invalid_bounds_indicies if i <= self.chosen_trajectory])
                        pths[i].poses = self.inbounds.trajectories[i-remove].poses
                        # green
                        tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
                        thickness = 5.0
                    elif i == len(pths) - 2:
                        tcols = Color(r=0.0, g=255.0, b=255.0, a=1.0)
                        thickness = 3.0
                    # center line
                    elif i == len(pths) - 1:
                        # blue
                        tcols = Color(r=0.0, g=0.0, b=255.0, a=1.0)
                        thickness = 3.0
                    # out of bounds
                    elif i in self.invalid_bounds_indicies:
                        appeneded += 1
                        # red 
                        tcols = Color(r=255.0, g=0.0, b=0.0, a=0.5)
                        thickness = 1.0
                    # in bounds
                    else:
                        # change trajectory to shorten trajectory if applicable
                        pths[i].poses = self.inbounds.trajectories[i-appeneded].poses
                        tcols = Color(r=255.0, g=255.0, b=255.0, a=0.8)
                        thickness = 1.0
                except:
                    print("stop here")

                points = []
                for j in range(len(pths[i].poses)):
                    # get a particular pose
                    _ = pths[i].poses[j].position
                    points.append(_)
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
                        'lifetime': Duration(sec=1,nanosec=0),
                        'frame_locked': False,
                        'lines': line_list,
                        'spheres': self.next_destination_vis}
            # scene update is a wrapper for scene entity
            scene_update_msg = SceneUpdate(entities=[SceneEntity(**sargs)])

            self.pubviz.publish(scene_update_msg)
            self.get_logger().info("Published msg")

            self.id += 1
            return
                
        self.get_logger().info("attributes not initialized")
        return

    # def show_chosen_paths(self, msg: PoseArray):
    #     tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
    #     pts = []
    #     line_list = [];
    #     for j in range(len(msg.poses)):
    #         # get a particular pose
    #         _ = msg.poses[j].position
    #         pts.append(_)
    #     args = {'type': LinePrimitive.LINE_STRIP,
    #             'pose': Pose(position=Point(x=0.0, y=0.0, z=0.0),
    #                          orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)),
    #             'thickness': 2.0,
    #             'scale_invariant': True,
    #             'points': pts,
    #             'color': tcols}
    #     line_list.append(LinePrimitive(**args))

        # arrow primitive code if needed
        # args = {'pose': Pose(position=Point(x=1.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
        #         'shaft_length': 1.0,
        #         'shaft_diameter': 0.1,
        #         'head_length': 2.5,
        #         'head_diameter': 0.5,
        #         'color': Color(r=67.0,g=125.0,b=100.0,a=1.0)}
        # msg = ArrowPrimitive(**args)

        # scene entity encapsulates these primitive objects
        sargs = {'timestamp': Time(sec=0,nanosec=0),
                    'frame_id': 'global_frame',
                    'id': f'{self.id}',
                    'lifetime': Duration(sec=3,nanosec=0),
                    'frame_locked': False,
                    'lines': line_list,
                    'spheres': self.next_destination_vis}

        # scene update is a wrapper for scene entity
        scene_update_msg = SceneUpdate(entities=[SceneEntity(**sargs)])

        self.pubviz.publish(scene_update_msg)
        self.get_logger().info("Published msg")

        self.id += 1

    def save_next_destination(self, msg : Pose):
        tcols = Color(r=255.0, g=255.0, b=0.0, a=1.0)
        args = {'pose': msg,
                'size': Vector3(x=1.0, y=1.0, z=1.0),
                'color': tcols}
        if len(self.next_destination_vis) == 0:
            self.next_destination_vis.append(SpherePrimitive(**args))
        else:
            self.next_destination_vis[0] = SpherePrimitive(**args)


def main():
    rclpy.init()
    nde = pub_viz()
    rclpy.spin(nde)
    nde.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()