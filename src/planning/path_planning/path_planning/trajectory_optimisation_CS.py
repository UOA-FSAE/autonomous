#!/usr/bin/python3
import numpy as np
import scipy.integrate
import scipy.interpolate
from shapely import LineString, MultiPoint
from shapely import Point as shapelyPoint
import scipy
import matplotlib.pyplot as plt
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Header, Float32, Int16, Int32MultiArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, Point
from moa_msgs.msg import Cone, ConeMap, BoundaryStamped, AllTrajectories, AllStates
from ackermann_msgs.msg import AckermannDrive
from builtin_interfaces.msg import Time


class trajectory_optimization(Node):
    def __init__(self):
        super().__init__("trajectory_optimisation")
        self.get_logger().info("Trajectory Optimisation Node Started")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('delete', True),
                ('interpolate', False)
            ]
        )

        # attributes
        self._once = True
        self._delete = self.get_parameter("delete").get_parameter_value().bool_value
        self._interpolate = self.get_parameter("interpolate").get_parameter_value().bool_value

        # subscribers
        self.create_subscription(AllStates, "moa/states", self.set_states, 10)
        self.create_subscription(AllTrajectories, "moa/trajectories", self.set_generated_trajectories, 10)
        self.create_subscription(ConeMap, "cone_map", self.callback, 10)
        # self.create_subscription(AckermannDrive, "moa/cur_vel", self.set_current_speed, 10)

        # publishers
        self.best_trajectory_publisher = self.create_publisher(PoseArray, "moa/selected_trajectory", 10)
        self.within_boundary_trajectories_publisher = self.create_publisher(AllTrajectories, 'moa/inbound_trajectories', 10)
        self.best_trajectory_index = self.create_publisher(Int16, "moa/best_trajectory_index", 10)
        self.out_of_bounds_indicies = self.create_publisher(Int32MultiArray, "moa/out_of_bounds", 10)
        self.best_steering_angle_pub = self.create_publisher(Float32, "moa/selected_steering_angle", 10)


    def set_states(self, msg: AllStates) -> None: self._state_msg = msg

    def set_generated_trajectories(self, msg: AllTrajectories) -> None: 
        self._trajectories_msg = msg
        self.get_logger().info(f"traj len = {len(msg.trajectories)}")

    def callback(self, msg:ConeMap) -> None:
        '''retrives cone msg and find trajectory closest to centerline'''

        if hasattr(self, "_state_msg") and hasattr(self, "_trajectories_msg"):
            self.get_logger().info(f"all states: {hasattr(self,'_state_msg')}"
                            f" | current speed: {hasattr(self,'_current_speed')}" \
                            f" | left boundaries: {hasattr(self,'_leftboundary')}"\
                            f" | right boundaries: {hasattr(self,'_rightboundary')}")   
            
            # yup = 1
            # xup = -0.5
            # count = 0
            # while count < 10:
                # msg.cones[0].pose.pose.position.x += xup
                # msg.cones[0].pose.pose.position.y += yup

            leftboundary, rightboundary, car_position = self.get_boundaries(msg.cones)    # get boundaries.
            if len(leftboundary) < 2:
                return
            if len(rightboundary) < 2:
                return
            position_orientation = self.get_position_of_cart(msg)
            if self._interpolate:
                leftboundary, rightboundary = self.get_relative_boundaries(leftboundary, rightboundary, car_position, position_orientation) # get local boundaries

            # get center line
            centerline = self.get_center_line(leftboundary, rightboundary, interpolate=self._interpolate)
            # x1, y1, x2, y2 = leftboundary[0][0], leftboundary[0][1], rightboundary[0][0], rightboundary[0][1]
            
            # adjust boundaries
            self._leftboundary = leftboundary
            self._rightboundary = rightboundary
            self._centerline = centerline

            # plot track
            if self._once:
                # plot
                self.plot_track(leftboundary,rightboundary,centerline,car_position)
                self._once = False
            
            if self._state_msg.id == self._trajectories_msg.id: # check if the steering angle (states) and generated trajectories are the same
                states = [ackerman_msg.steering_angle for ackerman_msg in self._state_msg.states]
                trajectories = self._trajectories_msg.trajectories.copy()   # COPY THIS! else errors will pop up like centerline added in trajectories


                if self._delete:
                    # self.get_logger().info(f"trajectories before deletion = {len(trajectories)}")
                    self.trajectory_deletion(trajectories, states)  # delete invalid trajectories
                    # self.get_logger().info(f"number of paths after deletion = {len(trajectories)}")

                best_trajectory_idx = self.optimisation(trajectories)   # find best/optimised trajectory
                
                if best_trajectory_idx == None: # check if no trajectory found
                    self.get_logger().info("no valid trajectories found")
                else:    
                    # self.get_logger().info(f"best indx = {best_trajectory_idx}")
                    # self.get_logger().info(f"best steering angle is = {states[best_trajectory_idx]}")

                    # publish best trajectory
                    args1 = {"header": Header(stamp=Time(sec=0,nanosec=0), frame_id='path_optimisation'),
                            "poses": trajectories[best_trajectory_idx].poses}
                    # publish valid (within boundaries) trajectories including center line
                    ps = [Pose(position=Point(x=P[0], y=P[1], z=0.0)) for P in centerline]
                    trajectories.append(PoseArray(poses=ps))
                    args2 = {"id": self._trajectories_msg.id, "trajectories": trajectories}
                    args3 = {"data": states[best_trajectory_idx]}   # float32 msg

                    # publish msgs
                    self.best_trajectory_publisher.publish(PoseArray(**args1))   # best trajectory pub
                    self.within_boundary_trajectories_publisher.publish(AllTrajectories(**args2)) # within bound pub
                    self.best_steering_angle_pub.publish(Float32(**args3)) # optimal steering angle pub

                    # self.get_logger().info("OPTIMAL TRAJECTORY COMPUTED")
            else:
                self.get_logger().info(f"Ids state:{self._state_msg.id} and trajectory:{self._trajectories_msg.id} do not match")
                
                # count += 1


    def get_boundaries(self, cones):
        leftboundary = []
        rightboundary = []
        for i in range(len(cones)):
            x = cones[i].pose.pose.position.x   # x point
            y = cones[i].pose.pose.position.y   # y point
            if i != 0:
                if cones[i].colour == 0:    # 0 is blue which is left
                    leftboundary.append([x,y])
                elif cones[i].colour == 2:  # 2 is yellow which is right
                    rightboundary.append([x,y])
            else:
                car_position = [x,y]    # first cone is car position
        
        return leftboundary, rightboundary, car_position
    
    def get_relative_boundaries(self, leftboundary, rightboundary, car_position, position_orientation):
        leftboundary = np.array(leftboundary).copy()
        rightboundary = np.array(rightboundary).copy()
        # position_vector, rotation_matrix = self.get_transformation_matrix(position_orientation)
        xc, yc = car_position
        see_ahead = 2
        leftboundary_distance = [0]*len(leftboundary)
        rightboundary_distance = [0]*len(rightboundary)
        # two for loops cuz lists are not same length
        for i, P in enumerate(leftboundary):    # go through all leftboundary points
            xl, yl = P  # global points
            # xl, yl = self.apply_transformation(position_vector, rotation_matrix, xl, yl) # get local point
            leftboundary_distance[i] = self.get_distance(xc, yc, xl, yl) # distance between car and left boundary point i
        for i, P in enumerate(rightboundary):
            xr, yr = P  # global points
            # xr, yr = self.apply_transformation(position_vector, rotation_matrix, xr, yr) # get local point
            rightboundary_distance[i] = self.get_distance(xc, yc, xr, yr)   # distance between car and right boundary point i
        
        # SIM GIVES OUR UNSORTED BOUNDARY POINTS!!
        closest_indices_l = np.sort(np.argsort(leftboundary_distance)[:see_ahead+1]).tolist() # indices of leftboundary points closest to car
        closest_indices_r = np.sort(np.argsort(rightboundary_distance)[:see_ahead+1]).tolist() # indices of rightboundary points closest to car
        if max(closest_indices_l) <= len(rightboundary)-1:  # index exists in both boundaries
            idx_range = closest_indices_l
        else:
            idx_range = closest_indices_r
        # vals = np.argmin(np.array(leftboundary_distance)), np.argmin(np.array(rightboundary_distance))
        self.get_logger().info(f"vals = {idx_range}")
        # start = max(vals)
        # end = start+see_ahead+1

        return leftboundary[idx_range], rightboundary[idx_range]

    def get_center_line(self, leftboundary, rightboundary, interpolate:False):
        '''approximates the track's center line'''
        # the midpoint is the average of the coordinates
        coods = []
        xps = []
        yps = []
        num_cones = min(len(leftboundary), len(rightboundary))
        for i in range(num_cones):
            x1, y1 = leftboundary[i]
            x2, y2 = rightboundary[i]
            x, y = self.get_avg_point(x1,y1,x2,y2)
            xps.append(x)
            yps.append(y)
            coods.append([x,y])

        # interpolate center line
        if interpolate:
            if num_cones >= 3:  # min points for quad/cubic interp
                radius = self.get_arc_radius(coods[0], coods[1], coods[2])
                # self.get_logger().info(f"RADIUS = {radius}")
                # if radius <= 1000:  # only interp on corners not straights
                coods = []
                f = scipy.interpolate.interp1d(xps, yps, kind='quadratic')
                xrange = np.linspace(min(xps), max(xps), num=50)
                for X in xrange:
                    coods.append([X, float(f(X))])
            else:
                # linear interp - but how and needed?
                pass

        return coods

    def get_arc_radius(self,p1,p2,p3):
        """
        radius using 3 points - arc approximation
        """
        x1,y1 = p1
        x2,y2 = p2
        x3,y3 = p3
        a = self.get_distance(x1,y1, x2,y2)
        b = self.get_distance(x2,y2, x3,y3)
        c = self.get_distance(x3,y3, x1,y1)

        # Calculate half perimeter
        s = (a + b + c) * 0.5
        
        # Heron's formula for triangle area
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))

        # Calculate radius using Menger curvature formula
        r = (a * b * c) / (4 * area)
        arc_distance = r* 2 * np.arcsin(b / (2 * r))

        # if curvature representative of path
        if abs(b-arc_distance) <= 5e-3:
            return r
        else:
            return 0

    def plot_track(self,leftboundary,rightboundary,centerline,car_position):
        plt.plot([P[0] for P in leftboundary], [P[1] for P in leftboundary], "ob", label='leftboundary')
        plt.plot([P[0] for P in rightboundary], [P[1] for P in rightboundary], "oy", label='rightboundary')
        plt.plot([P[0] for P in centerline], [P[1] for P in centerline], "ok", label='centerline')
        plt.plot([car_position[0]],[car_position[1]],'or', label='car position')
        # annotate points
        for i in range(len(leftboundary)):
            plt.annotate(f"{i}", leftboundary[i], textcoords='data')
        for i in range(len(rightboundary)):
            plt.annotate(f"{i}", rightboundary[i], textcoords='data')
        plt.grid()
        plt.legend()
        plt.show()

    def get_shapely_linestring(self, points) -> LineString:
        '''create a shapely linestring from an array/list of [x,y] points'''
        return LineString([(P[0], P[1]) for P in points])
    
    def get_avg_point(self, x1, y1, x2, y2):
        return [((x1 + x2)/2), ((y1 + y2)/2)]
    
    def get_position_of_cart(self, cone_map):
        # first cone
        localization_data = cone_map.cones[0]
        x = localization_data.pose.pose.position.x
        y = localization_data.pose.pose.position.y
        theta = localization_data.pose.pose.orientation.w
        return x, y, theta

    def get_transformation_matrix(self, position_and_orientation):
        # theta = position_and_orientation[2] - np.pi/2
        cart_x = position_and_orientation[0]
        cart_y = position_and_orientation[1]
        theta = position_and_orientation[2]
        # 2d trasformation matrix 
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        position_vector = np.array([[cart_x], [cart_y]])

        return position_vector, rotation_matrix

    def apply_transformation(self, position_vector, rotation_matrix, point_x, point_y):
        point = np.array([[point_x], [point_y]])
        # matrix multiplication for rotation then translate from car position
        transformed_point = np.matmul(rotation_matrix, point) - position_vector
        
        return transformed_point[0][0], transformed_point[1][0]

    ''' ---------------------------------- DELETION ---------------------------------- '''
    def trajectory_deletion(self, trajectories, states):
        '''Caller for trajectory deletion if there are trajectories'''

        self.get_logger().info("Trajectory Deletion Started")
        self.set_within_boundary_trajectories(trajectories, states) 
        
    def set_within_boundary_trajectories(self, trajectories, states):
        '''Deletes trajectories that are on track boundaries'''
        remove_trajectories_indices = []
        
        # if shapely code fails use this below (not 100%)
        # for i, T in enumerate(trajectories):
        #     if i == 25:
        #         print("stop here")
        #     for cpose in T.poses:
        #         x = cpose.position.x 
        #         y = cpose.position.y
        #         # compare points with boundaries
        #         onBound = self.compare_with_boundary(x,y,2)
        #         if onBound:  
        #             rm_inds.append(i)
        #             # trajectories.pop([i for i in range(len(trajectories)) if T==trajectories[i]][0])
        #             break

        left_boundary_linestring = self.get_shapely_linestring(self._leftboundary)
        right_boundary_linestring = self.get_shapely_linestring(self._rightboundary)

        for i in range(len(trajectories)):  # loop each trajectory
            trajectory = self.get_shapely_linestring([[P.position.x, P.position.y] for P in trajectories[i].poses])
            num_poses = len(trajectories[i].poses)

            # check trajectory intersection
            ips = None  # ips = intersection points
            if trajectory.intersects(left_boundary_linestring):   # left bound intersection
                # get intersection point/s
                ips = trajectory.intersection(left_boundary_linestring)
                intersection = "left"
            elif trajectory.intersects(right_boundary_linestring):    # right bound intersection
                ips = trajectory.intersection(right_boundary_linestring)
                intersection = "right"

            if ips is not None:
                remove_pose_indices = []    # poses to remove (index)
                if type(ips) is MultiPoint:
                    tmp_x = ips.centroid.x
                    tmp_y = ips.centroid.y
                else:
                    tmp_x = ips.x
                    tmp_y = ips.y

                for j, P in enumerate(trajectories[i].poses):   # loop through trajectory i
                    if num_poses >= 2: 
                        in_bounds = self.is_within_boundaries(inter=intersection,x1=tmp_x,y1=tmp_y,x2=P.position.x,y2=P.position.y) # get if pose outside bounds
                        if not in_bounds:
                            remove_pose_indices.append(j)   # if pose outside 
                    else:
                        remove_trajectories_indices.append(i)   # remove trajectory and stop
                        break
                
                [trajectories[i].poses.pop(index) for index in list(reversed(remove_pose_indices))] # remove out of bound poses for trajectory i

        # publish indicies
        print(f"removed indices = {remove_trajectories_indices}")
        self.out_of_bounds_indicies.publish(Int32MultiArray(data=remove_trajectories_indices))

        # remove out of bound trajectories
        [(trajectories.pop(index), states.pop(index)) for index in list(reversed(remove_trajectories_indices))]

    def get_distance(self,x1,y1,x2,y2):
        return np.sqrt(((x2-x1)**2 + (y2-y1)**2))
    
    def is_within_boundaries(self,inter,x1,y1,x2,y2):
        if inter == "right":
            return x2<x1 and y2<y1
        elif inter == "left":
            return x2>x1 and y2<y1
    ''' ---------------------------------- DELETION ---------------------------------- '''

    ''' ---------------------------------- OPTIMISATION ---------------------------------- '''
    def optimisation(self, trajectories):
        '''returns the best trajectory'''

        self.get_logger().info("Optimisation Started")
        return self.get_best_trajectory_state(trajectories) 

    def get_best_trajectory_state(self, trajectories):
        '''finds the best trajectory from the set'''

        trajectory_distances = np.ones(len(trajectories)) * np.inf
        # get center line
        center_linestring = LineString(self._centerline)

        for i in range(len(trajectories)):  # loop through each trajectory
            # trajectory = self.get_shapely_linestring(trajectories[i].poses) # trajectory i as a line
            P = trajectories[i].poses[-1].position  # end point of trajectory i
            xp, yp = [P.x, P.y]
            # end_point = shapelyPoint(xp,yp)   # as a point
            to_centerpoint_dist = []
            for P in self._centerline:
                cx,cy = P
                to_centerpoint_dist.append(self.get_distance(xp,yp,cx,cy))
            dist = min(to_centerpoint_dist)
            trajectory_distances[i] = dist # distance to centerline

        return self.get_best_trajectory_index(trajectory_distances)

    def get_best_trajectory_index(self, trajectory_distances): 
        idx = None
        try:
            objective_function = trajectory_distances 
            idx = int(np.argmin(objective_function))
            if idx > 398:
                print("stop here")
            self.best_trajectory_index.publish(Int16(data=idx))
        except ValueError:
            self.get_logger().info("error calculating objective value")

        return idx
    ''' ---------------------------------- OPTIMISATION ---------------------------------- '''

''' DEPRECATED/UNUSED CODE ATM BUT MAYBE NEEDED IN THE FUTURE '''
    # def get_left_boundary(self, rightboundary, track_width):
    #     angle = -90 * np.pi / 180
    #     leftboundary = []
    #     # loop through all left boundary points
    #     for i in range(len(rightboundary)):
    #         P = rightboundary[i]
    #         # check what point is
    #         if i == 0:
    #             # forward point
    #             vector = self.get_vector(rightboundary[i], rightboundary[i+1], True)
    #         elif i == len(rightboundary)-1:
    #             # backward point
    #             vector = self.get_vector(rightboundary[i-1], rightboundary[i], True)
    #         else:
    #             # centeral points
    #             vector = self.get_vector(rightboundary[i-1], rightboundary[i+1], True)

    #         # rotate vector 
    #         vector = self.get_rotated_vector(angle,vector)
    #         # compute new point
    #         new_point = P + track_width * vector
    #         # append new point
    #         leftboundary.append(new_point)

    #     return leftboundary

    # def get_vector(self, p1, p2, unit=True):
    #     '''calculates point vector based on slope'''
    #     vector = np.array(p2)-np.array(p1)
    #     if unit:
    #         return vector / self.get_magnitude(vector)
        
    #     return vector
    
    # def get_rotated_vector(self, angle, vector):
    #     # rotation matrix
    #     rotate = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

    #     return np.dot(rotate,np.array(vector))
    
    # def get_magnitude(self, vector):
        # return np.sqrt(sum(vector**2))

    # def compare_with_boundary(self, x, y, tol):
    #     '''
    #     Compares a coordinate with the left and right boundary coordinates of the track map

    #     inputs
    #         x (float): x position of coordinate
    #         y (float): y position of coordinate
    #     return
    #         (boolean): True if point on either boundary
        
    #     * Assumes same number of points are given for left and right boundary
    #     '''
    #     lxyt = []
    #     rxyt = []
    #     for i in range(len(self.rightbound)):
    #         blx, bly = self.leftbound[i]
    #         brx, bry = self.rightbound[i]
    #         # on left boundary
    #         if i != 0:
    #             # lxyt = abs(x-blx) <= 1e-1 and abs(y-bly) <= 1e-1
    #             lxyt.append(np.sqrt(((x-blx)**2 + (y-bly)**2)))
    #         else:
    #             lxyt.append(np.inf)
    #         # on right boundary
    #         # rxyt = abs(x-brx) <= 1e-1 and abs(y-bry) <= 1e-1
    #         rxyt.append(np.sqrt(((x-brx)**2 + (y-bry)**2)))
    #     # check if near boundary
    #     if min(lxyt) <= tol or min(rxyt) <= tol:
    #         return True
    #     else:
    #         return False
        
def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    node = trajectory_optimization()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()








