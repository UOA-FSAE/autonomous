import rclpy
from rclpy.node import Node

from moa_msgs.msg import ConeMap
from geometry_msgs.msg import Pose

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

class centerline_planner(Node):
    def __init__(self):
        super().__init__("centerline_planner")

        # parameters
        self._plot = True

        # subscribers
        self.create_subscription(ConeMap, "cone_map", self.callback, 10)    # track
        self.create_subscription(Pose, "car_position", self.set_car_position, 10)   # car pose

        # publishers
    
    def set_car_position(self, msg:Pose) -> None: 
        self.car_pose = msg
    
    def callback(self, msg:ConeMap) -> None:
        lb, rb = self.get_boundaries(msg)   # get boundaries (list of [x,y] points)

        centerline = self.compute_centerline(lb,rb) # compute centerline

        centerline = self.interpolate_line(centerline) # interpolate/smooth

        # publish

        self.plot(lb,rb,centerline,self._plot) # plot
    
    # MAIN FUNCTIONS
    def get_boundaries(self,msg:ConeMap):
        """Retrieves the left and right boundary from msg and returns as a [x,y] list"""
        left_cones = [[P.x, P.y] for P in msg.left_cones]
        right_cones = [[P.x, P.y] for P in msg.right_cones]

        return left_cones, right_cones
    
    def compute_centerline(self,lb,rb):
        """Computes the centerline using given boundary points based on closest distance
           boundary points do not need to be sorted
        """
        centerline = []

        for P in lb:    # loop through left boundary
            closest_point = self.get_closest_point(P,rb)    # get the point closest in the right boundary
            centerpoint = (P+closest_point)/2 # midpoint
            centerline.append(centerpoint)
        
        return centerline
    
    def interpolate_line(self,line):
        line = self.univariate_interpolate(line,300,5)

        return line
    
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
            plt.plot(centx,centy,'*r',label='centerline')
            plt.plot(car_x,car_y,'*k',label='car position')

            plt.pause(0.1)
            plt.legend()
            plt.show()
    
    # HERLPERS FUNCTIONS
    def get_closest_point(self,point,points):
        point = np.array(point)
        points = np.array(points)

        distances = np.linalg.norm(point - points, axis=1)

        return points[np.argmin(distances)]

    def univariate_interpolate(self,line,num_points,smoothness):
        line = np.array(line)
        x,y = line[:,0], line[:,1]
        t = range(len(x))  # t is for defining x and y parametrically

        spline_x = UnivariateSpline(t,x,s=smoothness)
        spline_y = UnivariateSpline(t,y,s=smoothness)

        t_new = np.linspace(min(t),max(t),num_points)

        x = spline_x(t_new)
        y = spline_y(t_new)
    
        return list(zip(x,y))


def main(args=None):
    rclpy.init(args=args)

    node = centerline_planner()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        