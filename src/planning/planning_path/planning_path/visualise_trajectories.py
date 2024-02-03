#!/usr/bin/python3
from foxglove_msgs.msg import LinePrimitive, Color, SceneEntity, SceneUpdate, ArrowPrimitive, PoseInFrame, PosesInFrame
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3, Quaternion 
from moa_msgs.msg import AllTrajectories, AllStates
from ackermann_msgs.msg import AckermannDrive
from builtin_interfaces.msg import Time, Duration
import rclpy
from rclpy.node import Node
import numpy as np

class pub_viz(Node):
    def __init__(self):
        super().__init__("publish_path_planning_msgs")
        self.get_logger().info("path planning visulisation node started")

        self.pubviz = self.create_publisher(SceneUpdate, 'visualization_trajectories', 5)
        # sub to all trajectories points and states
        self.all_paths = self.create_subscription(AllTrajectories, "moa/inbound_trajectories", self.show_paths, 5)
        # self.all_paths = self.create_subscription(AllTrajectories, "moa/trajectories", self.show_paths, 5)
        self.all_states = self.create_subscription(AllStates, "moa/inbound_states", self.get_all_states, 5)
        # selected path
        self.chosen_states = self.create_subscription(AckermannDrive, "moa/selected_trajectory", self.get_chosen_state_idx, 5)

        self.id = 1

    def get_all_states(self, msg:AllStates) -> None: self.states = [i.steering_angle for i in msg.states]

    def get_chosen_state_idx(self, msg:AckermannDrive) -> None: 
        if hasattr(self, "states"): 
            self.chosen_idx = np.where(np.isclose(self.states, msg.steering_angle, 1e-3))[0][-1]

    def show_paths(self, msg: AllTrajectories):
        if not hasattr(self,"chosen_idx"):
            self.get_logger().info("attribute not initialized")
            return
        
        line_list = []
        # list of pose array
        pths = msg.trajectories
        for i in range(len(pths)):
            if i == self.chosen_idx:
                tcols = Color(r=255.0, g=255.0, b=255.0, a=1.0)
            elif i == len(pths) - 1:
                tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
            else:
                tcols = Color(r=255.0, g=0.0, b=0.0, a=1.0)
            pts = []
            for j in range(len(pths[i].poses)):
                # get a particular pose
                _ = pths[i].poses[j].position
                pts.append(_)
            args = {'type': LinePrimitive.LINE_STRIP,
                    'pose': Pose(position=Point(x=0.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
                    'thickness': 2.0,
                    'scale_invariant': True,
                    'points': pts,
                    'color': tcols}
            line_list.append(LinePrimitive(**args))

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
                    'lines': line_list}
        # scene update is a wrapper for scene entity
        scene_update_msg = SceneUpdate(entities=[SceneEntity(**sargs)])

        self.pubviz.publish(scene_update_msg)
        self.get_logger().info("Published msg")

        self.id += 1

def main():
    rclpy.init()
    nde = pub_viz()
    rclpy.spin(nde)
    nde.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()