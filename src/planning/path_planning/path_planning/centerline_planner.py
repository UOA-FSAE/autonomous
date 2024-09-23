import rclpy
from rclpy.node import Node

from moa_msgs.msg import Track
from geometry_msgs.msg import Pose, PoseArray

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

class centerline_planner(Node):
    def __init__(self):
        super().__init__("centerline_planner")

        # parameters
        self._plot = True
        self.look_forward = 4
        self.num_points = self.look_forward*2
        self.smoothing_factor = 1
        self.left_cones = []
        self.right_cones = []

        # subscribers
        self.subscription_left_cone_map = self.create_subscription(Track, 'left_track',  self.left_cone_map_callback, 10)
        self.subscription_right_cone_map = self.create_subscription(Track, 'right_track',  self.right_cone_map_callback, 10)
        self.create_subscription(Pose, "car_position", self.set_car_position, 10)   # car pose

        # publishers
        self.centerline_publisher = self.create_publisher(PoseArray, "moa/selected_trajectory", 10)
    
    def set_car_position(self, msg:Pose) -> None: 
        self.car_pose = msg
        self.loop()

    def left_cone_map_callback(self, msg:Track) -> None:
        self.left_cones = msg.cones

    def right_cone_map_callback(self, msg:Track) -> None:
        self.right_cones = msg.cones
    
    def loop(self) -> None:
        self.get_logger().info(f"car pose recieved = {hasattr(self,'car_pose')}")   
        
        if not hasattr(self,'car_pose'):
            return 
        
        lb, rb = self.get_boundaries()   # get boundaries (list of [x,y] points)

        if not len(lb) > 0 or not len(rb) > 0:
            return

        lblocal, _ = self.get_next_points(lb,self.look_forward)  # get local points (list of [x,y] points) close to car
        rblocal, car_position = self.get_next_points(rb,self.look_forward)

        if not len(lblocal) > 0 or not len(rblocal) > 0:
            return

        centerline = self.compute_centerline(lblocal,rblocal,car_position) # compute centerline

        if not len(centerline) > 0:
            return

        centerline = self.interpolate_line(centerline,self.num_points,self.smoothing_factor) # interpolate/smooth

        # publish
        msg = self.get_posearray_msg(centerline)
        self.centerline_publisher.publish(msg)

        self.plot(lb,rb,centerline,self._plot) # plot
    
    # MAIN FUNCTIONS
    def get_boundaries(self):
        """Retrieves the left and right boundary from msg and returns as a [x,y] list"""
        left_cones = [[P.x, P.y] for P in self.left_cones]
        right_cones = [[P.x, P.y] for P in self.right_cones]

        return left_cones, right_cones

    def get_next_points(self,line,look_forward):
        """Retrieves the points closest to the car
        *ASSUMES THE points ARE SORTED/ORDERED
        """
        car_point = np.array([self.car_pose.position.x,self.car_pose.position.y])
        car_orientation = np.array([self.car_pose.orientation.x,self.car_pose.orientation.y,
                                    self.car_pose.orientation.z,self.car_pose.orientation.w])
        points = np.array(line)
        
        min_indx = np.argmin(np.linalg.norm(car_point-points, axis=1))
        # min_indx += self.is_behind_car(points[min_indx], car_point, car_orientation)
        points = self.get_local_points(min_indx,points,look_forward)

        return points, car_point
    
    def compute_centerline(self,lb,rb,car_position):
        """Computes the centerline using given boundary points based on closest distance
           boundary points do not need to be sorted
           Also adds car position at the beginning of the centerline
        """
        centerline = np.array([car_position])

        for P in lb:    # loop through left boundary
            closest_point = self.get_closest_point(P,rb)    # get the point closest in the right boundary
            centerpoint = (P+closest_point)/2 # midpoint
            centerline = np.append(centerline,[centerpoint],axis=0)
        
        return centerline
    
    def interpolate_line(self,line,num_points=100,smoothing_factor=1):
        line = self.univariate_interpolate(line,num_points,smoothing_factor)

        return line
    
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

            plt.plot(lbx,lby,'*y',label='left')
            plt.plot(rbx,rby,'*b',label='right')
            plt.plot(centx,centy,'-r',label='centerline')
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
        if len(line) < 4:
            return line
        
        line = np.array(line)
        x,y = line[:,0], line[:,1]
        t = range(len(x))  # t is for defining x and y parametrically
        weights = np.ones(len(x))   # second point should be weighted less (to minimise if its behind)
        weights[1] = 0.1

        spline_x = UnivariateSpline(t,x,w=weights,s=smoothness)
        spline_y = UnivariateSpline(t,y,w=weights,s=smoothness)

        t_new = np.linspace(min(t),max(t),num_points)

        x = spline_x(t_new)
        y = spline_y(t_new)
    
        return np.array(list(zip(x,y)))
    
    def is_behind_car(self, closet_point, car_point, car_orientation):
        from_car_point_to_closest_point = closet_point-car_point    # vector from car to closest point
        car_direction_vector = self.get_car_direction(car_orientation)
        if (from_car_point_to_closest_point @ car_direction_vector) < 0:
            return True
        else:
            return False
    
    def get_car_direction(self, orientation):
        rotation_matrix = self.get_rotation_matrix(orientation[3])

        return rotation_matrix @ np.array([1,0])
    
    def get_local_points(self,min_indx,points,look_forward):
        remaining_points = len(points) - (min_indx+1) # number of points forwards the car has detected
        if remaining_points < look_forward: look_forward = remaining_points+1
        points = points[min_indx:min_indx+look_forward] # local points based on closest distance to car

        return points
    
    def get_rotation_matrix(self,theta):
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])

        return rotation_matrix


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

        