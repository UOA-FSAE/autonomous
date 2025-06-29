import rclpy
from rclpy.node import Node

from moa_msgs.msg import Detections
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
        self.look_forward = 4
        self.left_cones = []
        self.right_cones = []
        self.path_planner = PathPlanner(MissionTypes.trackdrive)

        # subscribers
        self.subscription_cone_detection = self.create_subscription(Detections, 'cone_detection', self.cone_detection_callback, 10)
        self.create_subscription(Pose, "car_position", self.set_car_position, 10)   # car pose


        # publishers
        self.centerline_publisher = self.create_publisher(PoseArray, "selected_trajectory", 10)
    
    def set_car_position(self, msg:Pose) -> None: 
        self.car_pose = msg
        self.loop()

    def cone_detection_callback(self, msg: Detections) -> None:
        self.left_cones = msg.blue
        self.right_cones = msg.yellow
    
    def loop(self) -> None:
        self.get_logger().info(f"car pose recieved = {hasattr(self,'car_pose')}")   
        
        if not hasattr(self,'car_pose'):
            return 
        
        lb, rb = self.get_boundaries()   # get boundaries (list of [x,y] points)

        if not len(lb) > 0 or not len(rb) > 0:
            return

        lblocal, _, _= self.get_next_points(lb,self.look_forward)  # get local points (list of [x,y] points) close to car
        rblocal, car_position, car_orientation = self.get_next_points(rb,self.look_forward)

        if not len(lblocal) > 0 or not len(rblocal) > 0:
            return
        
        global_cones = self.get_global_cones(lblocal, rblocal)
        car_direction = self.get_car_direction(car_orientation)
        
        path = self.path_planner.calculate_path_in_global_frame(global_cones, car_position, car_direction) # compute centerline
        
        centerline = path[:, 1:3]

        if not len(centerline) > 0:
            return

        # publish
        msg = self.get_posearray_msg(centerline)
        self.centerline_publisher.publish(msg)

        self.plot(lb,rb,centerline,self._plot) # plot
        
        
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

    def get_next_points(self,line,look_forward):
        """Retrieves the points closest to the car
        *ASSUMES THE points ARE SORTED/ORDERED
        """
        car_point = np.array([self.car_pose.position.x,self.car_pose.position.y])
        car_orientation = self.car_pose.orientation.w
        points = np.array(line)
        
        min_indx = np.argmin(np.linalg.norm(car_point-points, axis=1))
        # min_indx += self.is_behind_car(points[min_indx], car_point, car_orientation)
        points = self.get_local_points(min_indx,points,look_forward)

        return points, car_point, car_orientation
    
    
    def get_posearray_msg(self, line):
        pose_array = PoseArray()

        for P in line:
            pose = Pose()
            pose.position.x = P[0]  # assing x,y values to position of pose
            pose.position.y = P[1]
            pose_array.poses.append(pose)
        
        return pose_array
    
    
    def plot(self,lb,rb,centerline,to_plot=False):
        """Plots the boundary and centerline points"""
        if to_plot:
            # x, y points
            lb = np.array(lb)
            rb = np.array(rb)
            centerline = np.array(centerline)

            lbx, lby = lb[:,0], lb[:,1]
            rbx, rby = rb[:,0], rb[:,1]
            centx, centy = centerline[:,0], centerline[:,1]
            car_x, car_y = [self.car_pose.position.x, self.car_pose.position.y]

            # plot
            plt.ion()
            plt.clf()

            plt.plot(lbx,lby,'*b',label='left')
            plt.plot(rbx,rby,'*y',label='right')
            plt.plot(centx,centy,'-r',label='centerline')
            plt.plot(car_x,car_y,'*k',label='car position')

            plt.pause(0.1)
            plt.legend()
            plt.show()
    
    
    def get_local_points(self,min_indx,points,look_forward):
        remaining_points = len(points) - (min_indx+1) # number of points forwards the car has detected
        if remaining_points < look_forward: look_forward = remaining_points+1
        points = points[min_indx:min_indx+look_forward] # local points based on closest distance to car

        return points


def main(args=None):
    rclpy.init(args=args)
    node = centerline_planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()