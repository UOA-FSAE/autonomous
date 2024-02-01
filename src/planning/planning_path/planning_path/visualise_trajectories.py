#!/usr/bin/python3
from foxglove_msgs.msg import LinePrimitive, Color, SceneEntity, SceneUpdate, ArrowPrimitive, PoseInFrame, PosesInFrame
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3, Quaternion 
from moa_msgs.msg import AllTrajectories
from builtin_interfaces.msg import Time, Duration
import rclpy
from rclpy.node import Node
import numpy as np

class pub_viz(Node):
    def __init__(self):
        super().__init__("publish_path_planning_msgs")
        self.get_logger().info("path planning visulisation node started")

        self.pubviz = self.create_publisher(SceneUpdate, 'trajectories_viz', 1)
        #sub to all trajectories points and callback func
        self.all_paths = self.create_subscription(AllTrajectories, "moa/trajectories", self.show_paths, 1)
        #TODO - sub to final trajectory POINT (HOW?) and save points 
        # self.create_timer(1.0,self.create_traj)

    def show_paths(self, msg: AllTrajectories):
        # pts = [Point(x=0.0,y=0.0,z=0.0),Point(x=2.0,y=0.0,z=0.0)]
        # xt = 2.0
        # yt = 0.0
        # for i in range(10):
        #     xt += 0.2
        #     yt += 0.1
        #     pts.append(Point(x=xt,y=yt,z=0.0))
        line_list = []
        # list of pose array
        pths = msg.trajectories
        for i in range(len(pths)):
            if i == 19:
                tcols = Color(r=255.0, g=255.0, b=255.0, a=1.0)
            else:
                tcols = Color(r=255.0, g=0.0, b=0.0, a=1.0)
            # cols = np.random.choice(range(256),3)
            pts = [Point(x=0.0,y=0.0,z=0.0)]
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

        # arrow primitive code
        # args = {'pose': Pose(position=Point(x=1.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
        #         'shaft_length': 1.0,
        #         'shaft_diameter': 0.1,
        #         'head_length': 2.5,
        #         'head_diameter': 0.5,
        #         'color': Color(r=67.0,g=125.0,b=100.0,a=1.0)} 
        # msg = ArrowPrimitive(**args)

        # args = {'timestamp':Time(sec=0,nanosec=0),
        #         'frame_id': 'global frame',
        #         'pose':Pose(position=Point(x=self.x,y=0.0,z=0.0), orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0))}

        # msg = PoseInFrame(**args)

        # scene entity encapsulates these primitive objects
        sargs = {'timestamp': Time(sec=0,nanosec=0),
                    'frame_id': 'global_frame',
                    'id': '100',
                    'lifetime': Duration(sec=3,nanosec=0),
                    'frame_locked': False,
                    'lines': line_list}
        # scene update is a wrapper for scene entity
        scene_update_msg = SceneUpdate(entities=[SceneEntity(**sargs)])

        self.pubviz.publish(scene_update_msg)
        self.get_logger().info("Published msg")

        # self.x += 0.1

def main():
    rclpy.init()
    nde = pub_viz()
    rclpy.spin(nde)
    nde.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()