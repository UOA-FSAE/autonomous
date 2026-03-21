import rclpy
from rclpy.node import Node

from fsae_interfaces.msg import Track
from geometry_msgs.msg import Pose, PoseArray

import numpy as np
import matplotlib.pyplot as plt
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
import math


class centerline_planner(Node):
    def __init__(self):
        super().__init__("centerline_planner")

        # parameters
        self._plot = True
        self.look_forward = 6
        self.left_cones = []
        self.right_cones = []
        self.path_planner = PathPlanner(MissionTypes.trackdrive)

        # subscribers
        self.subscription_left_cone_map = self.create_subscription(Track, 'left_track',  self.left_cone_map_callback, 10)
        self.subscription_right_cone_map = self.create_subscription(Track, 'right_track',  self.right_cone_map_callback, 10)
        self.create_subscription(Pose, "car_position", self.set_car_position, 10)   # car pose

        # publishers
        self.centerline_publisher = self.create_publisher(PoseArray, "selected_trajectory", 10)
        self.declare_parameters(
            namespace='',
            parameters=[
            ('invert_cones', False),
            ]
        )
    
    def set_car_position(self, msg: Pose) -> None: 
        self.car_pose = msg
        self.loop()

    def left_cone_map_callback(self, msg: Track) -> None:
        self.left_cones = msg.cones

    def right_cone_map_callback(self, msg: Track) -> None:
        self.right_cones = msg.cones
    
    def loop(self) -> None:
        self.get_logger().info(f"car pose received = {hasattr(self, 'car_pose')}")

        if not hasattr(self, 'car_pose'):
            return

        lb, rb = self.get_boundaries()   # get boundaries (list of [x,y] points)

        # >>> DEBUG override: when only one cone color is seen
        if len(lb) > 0 and len(rb) == 0:  # only blue cones
            self.get_logger().info("DEBUG: only blue cones -> full right")  # >>> LOG
            car_x = self.car_pose.position.x     # >>> CHANGED
            car_y = self.car_pose.position.y     # >>> CHANGED
            # simple right-turn trajectory
            right_line = [                      # >>> CHANGED
                [car_x + 1.0, car_y + 1.0],
                [car_x + 2.0, car_y + 2.0],
            ]
            msg = self.get_posearray_msg(right_line)  # >>> CHANGED
            self.centerline_publisher.publish(msg)     # >>> CHANGED
            return                                   # >>> CHANGED
        elif len(rb) > 0 and len(lb) == 0:  # only yellow cones
            self.get_logger().info("DEBUG: only yellow cones -> full left")   # >>> LOG
            car_x = self.car_pose.position.x     # >>> CHANGED
            car_y = self.car_pose.position.y     # >>> CHANGED
            # simple left-turn trajectory
            left_line = [
                [car_x - 1.0, car_y + 1.0],
                [car_x - 2.0, car_y + 2.0],
            ]
            msg = self.get_posearray_msg(left_line)   # >>> CHANGED
            self.centerline_publisher.publish(msg)     # >>> CHANGED
            return                                   # >>> CHANGED
        # <<< end DEBUG override >>>

        lblocal, _, _ = self.get_next_points(lb, self.look_forward)  # get local points (list of [x,y] points) close to car
        rblocal, car_position, car_orientation = self.get_next_points(rb, self.look_forward)

        if not len(lblocal) > 0 or not len(rblocal) > 0:
            return
        
        global_cones = self.get_global_cones(lblocal, rblocal)
        car_direction = self.get_car_direction(car_orientation)
        
        path = self.path_planner.calculate_path_in_global_frame(global_cones, car_position, car_direction)  # compute centerline
        
        centerline = path[:, 1:3]

        if not len(centerline) > 0:
            return

        # publish
        msg = self.get_posearray_msg(centerline)
        self.centerline_publisher.publish(msg)

        self.plot(lb, rb, centerline, self._plot)  # plot
        
    def get_global_cones(self, lb, rb):
        cones_by_type = [np.zeros((0, 2)) for _ in range(5)]
        cones_by_type[ConeTypes.LEFT] = lb
        cones_by_type[ConeTypes.RIGHT] = rb
        return cones_by_type
        
    def get_car_direction(self, car_orientation):
        return np.array([-math.sin(car_orientation), math.cos(car_orientation)])
        
    # MAIN FUNCTIONS
    def get_boundaries(self):
        """Retrieves the left and right boundary from msg and returns as a [x,y] list"""
        left_cones = np.array([[P.x, P.y] for P in self.left_cones])
        right_cones = np.array([[P.x, P.y] for P in self.right_cones])

        return left_cones, right_cones

    def get_next_points(self, line, look_forward):
        """Retrieves the points closest to the car
        *ASSUMES THE points ARE SORTED/ORDERED
        """
        car_point = np.array([self.car_pose.position.x, self.car_pose.position.y])
        car_orientation = self.car_pose.orientation.w
        points = np.array(line)
        
        min_indx = np.argmin(np.linalg.norm(car_point - points, axis=1))
        points = self.get_local_points(min_indx, points, look_forward)

        return points, car_point, car_orientation
        
    def get_posearray_msg(self, line):
        pose_array = PoseArray()

        for P in line:
            pose = Pose()
            pose.position.x = P[0]
            pose.position.y = P[1]
            pose_array.poses.append(pose)
        
        return pose_array
        
    def plot(self, lb, rb, centerline, to_plot=False):
        """Plots the boundary and centerline points"""
        if to_plot:
            lb = np.array(lb)
            rb = np.array(rb)
            centerline = np.array(centerline)

            lbx, lby = lb[:, 0], lb[:, 1]
            rbx, rby = rb[:, 0], rb[:, 1]
            centx, centy = centerline[:, 0], centerline[:, 1]
            car_x, car_y = [self.car_pose.position.x, self.car_pose.position.y]

            plt.ion()
            plt.clf()

            plt.plot(lbx, lby, '*b', label='left')
            plt.plot(rbx, rby, '*y', label='right')
            plt.plot(centx, centy, '-r', label='centerline')
            plt.plot(car_x, car_y, '*k', label='car position')
            if not self.get_parameter('invert_cones').get_parameter_value().bool_value:
                plt.plot(lbx,lby,'*b',label='left')
                plt.plot(rbx,rby,'*y',label='right')
            else:
                plt.plot(lbx,lby,'*y',label='right')
                plt.plot(rbx,rby,'*b',label='left')
                
            plt.plot(centx,centy,'-r',label='centerline')
            plt.plot(car_x,car_y,'*k',label='car position')

            plt.pause(0.1)
            plt.legend()
            plt.show()

    def get_local_points(self, min_indx, points, look_forward):
        remaining_points = len(points) - (min_indx + 1)
        if remaining_points < look_forward:
            look_forward = remaining_points + 1
        points = points[min_indx:min_indx + look_forward]

        return points


def main(args=None):
    rclpy.init(args=args)
    node = centerline_planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
