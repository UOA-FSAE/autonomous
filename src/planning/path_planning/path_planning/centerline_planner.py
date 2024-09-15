import rclpy
from rclpy.node import Node

from moa_msgs.msg import ConeMap

import numpy as np
import matplotlib.pyplot as plt

class centerline_planner(Node):
    def __init__(self):
        super().__init__("centerline_planner")

        # parameters
        self._plot = True
        plt.show()

        # subscribers
        self.create_subscription(ConeMap, "cone_map", self.callback, 10)

        # publishers
    
    def callback(self, msg:ConeMap) -> None:
        lb, rb = self.get_boundaries(msg)   # get boundaries (list of [x,y] points)

        centerline = self.compute_centerline(lb,rb) # compute centerline

        centerline = self.interpolate_line(centerline) # interpolate/smooth

        # publish

        self.plot(lb,rb,centerline,self._plot) # plot
    
    # MAIN FUNCTIONS
    def get_boundaries(msg:ConeMap):
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
            centerpoint = np.mean(P, closest_point) # midpoint
            centerline.append(centerpoint)
        
        return centerline
    
    def interpolate_line(line):
        return line
    
    def plot(lb,rb,centerline,to_plot):
        # x, y points
        lb = np.array(lb)
        rb = np.array(rb)
        centerline = np.array(centerline)

        lbx, lby = lb[:,0], lb[:,1]
        rbx, rby = rb[:,0], rb[:,1]
        centx, centy = centerline[:,0], centerline[:,1]

        # plot
        plt.plot(lbx,lby,'*b',label='left')
        plt.plot(rbx,rby,'*y',label='right')
        plt.plot(centx,centy,'*r',label='centerline')

        plt.plegend()
    
    # HERLPERS FUNCTIONS
    def get_closest_point(point, points):
        point = np.array(point)
        points = np.array(points)

        distances = np.linalg.norm(point - points, axis=1)

        return points[np.argmin(distances)]


def main(args=None):
    rclpy.init(args=args)

    centerline_planner = centerline_planner()

    rclpy.spin(centerline_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    centerline_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        