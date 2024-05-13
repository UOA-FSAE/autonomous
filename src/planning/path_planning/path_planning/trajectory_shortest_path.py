#!/usr/bin/python3
from path_planning.shortest_path.CoreModels import Node, State
import path_planning.shortest_path.TrackMethods as TrackMethods
import path_planning.shortest_path.TrackHelpers as TrackHelpers
import path_planning.shortest_path.PathHelpers as PathHelpers

import numpy as np
import pandas as pd
# from shapely import LineString, MultiPoint
# from shapely import Point as shapelyPoint
import os
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node as NODE
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Header, Float32, Int16, Int32MultiArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, Point
from moa_msgs.msg import Cone, ConeMap


class shortest_path(NODE):
    def __init__(self):
        super().__init__("shortest_path")
        self.get_logger().info("SHORTEST PATH STARTED")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('plot', True),
                ('save_track', False),
            ]
        )

        # attributes
        self._plot = self.get_parameter("plot").get_parameter_value().bool_value
        self._save_track = self.get_parameter("save_track").get_parameter_value().bool_value
        self._sim = False

        # car properties
        self._CAR = {
            "mass": 84.5,  # kg
            "μ": 0.6, # static friction coefficient - dimensionless
            "α": PathHelpers.noughtTo60(3.0), 
            "α_d": 15.0,    # max decel in m/s^2
            "max steer angle": 25.0, # degrees
            "max velocity": 10.0,  # m/s
            "tire width": 0.11, # in m 
            "wheelbase": 1.5,    # wheelbase length (in m)
        }

        # subscribers
        self.create_subscription(ConeMap, "cone_map", self.callback, 10)

        # publishers
        self.steering_angle = self.create_publisher(Float32, "/test/cmd_steering", 10)
        self.best_trajectory_publisher = self.create_publisher(PoseArray, "moa/selected_trajectory", 10)


    def callback(self, msg:ConeMap):
        innerboundary, outerboundary, car_position, w, oc, ic = self.get_boundaries(msg.cones)
        self.get_logger().info(f"car orientation = {w}")
        # saving track 
        if self._save_track:
            self.save_track(innerboundary, outerboundary)
        
        if self._sim:
            # FOR SOME REASON THE CONE LIST IN THE SIM COMES OUT AS OPPOSITE WITH CONES FURTHEST AWAY AT FIRST INDEX
            # DOEST HAPPEN WITH FAKE CONE DATA
            innerboundary.reverse()
            outerboundary.reverse()
        # smaller (incl. negative) x, y come before 
        innerboundary = np.array(innerboundary)
        outerboundary = np.array(outerboundary)

        # self.get_logger().info(f"bound: {innerboundary}")
        
        # you want pairs of cones for path planning to work - HAPPENDS IN SIM & FAKE CONE
        if len(innerboundary) > len(outerboundary):
            innerboundary = innerboundary[:len(outerboundary)]
        elif len(outerboundary) > len(innerboundary):
            outerboundary = outerboundary[:len(innerboundary)]
        
        if self._plot:
            xl = [P[0] for P in innerboundary]
            yl = [P[1] for P in innerboundary]
            xr = [P[0] for P in outerboundary]
            yr = [P[1] for P in outerboundary]
            plt.plot(xl, yl, "ob", label="left boundary")
            plt.plot(xr, yr, "og", label="right boundary")
            plt.plot(car_position[0],car_position[1], "or", label="car position")
            plt.legend()
            plt.show()

        # if ic > oc: innerboundary = innerboundary[:(oc-ic)] 
        # if oc > ic: outerboundary = outerboundary[:(ic-oc)]

        # transform the points
        # innerboundary = np.array(list(map(lambda P: self.get_transformed_point(msg, P), innerboundary)))
        # outerboundary = np.array(list(map(lambda P: self.get_transformed_point(msg, P), outerboundary)))
        # car_position = self.get_transformed_point(msg, car_position)
    
        # # get center line and track widths
        # center_line, track_widths = self.get_center_line(innerboundary, outerboundary)
        # # format as pandas data frame
        # track_info = self.get_track_info(center_line, track_widths)

        # get shortest path
        print("IMPORTING TRACK")
        # df = TrackMethods.importTrack(track_info=track_info, plot=self._plot)

        df = self.create_track_dataframe(innerboundary, outerboundary)  # both boundaries should be equal length

        # create brackets
        print("CREATING BRACKETS")
        brackets = TrackMethods.getBrackets(df, 5, plot=self._plot)

        # compute optimal path
        print("COMPUTING OPTIMAL PATH")
        # car_position = np.array(car_position) + (np.random.rand(2)*2)
        start_node = brackets[int(np.random.randint(0,5))]._nodeList[int(np.random.randint(0,6))]
        car_position = start_node._xy

        current_position, brackets = self.getStartingPosition(car_position, brackets)
        start_node = self.get_start_node(starting_point=current_position)   # transformed car position
        angle = np.pi / 2
        start_node._stateList.append(State(start_node, np.array([np.cos(angle), np.sin(angle)]), 0.0, np.Inf))
        print("starting inner distance: ", start_node._innerDistance)
        print("starting outer distance: ", start_node._outerDistance)
        # start_node = TrackMethods.belman_ford_path(df, velocity_range, brackets, start_node, plot=self._plot)
        n_vel = 10
        start_node, brackets, optimal_cost = TrackMethods.optimal_path(
            "$track_name optimal", 
            car_position,
            df, 
            start_node, 
            brackets, 
            n_vel,
            self._CAR,
            self._plot
        )
        print("\nOPTIMAL PATH COMPUTED")


        # get steering angle based on current and next point
        p1 = start_node._xy # relative to global
        # self.get_logger().info(f"{p1}")
        # self.get_logger().info(f"{start_node._nextNode._xy}")
        # # p2 = self.get_transformed_point(msg, start_node._nextNode._xy)   # relative to local
        p2 = start_node._stateList[0]._nextState._xy 
        # self.get_logger().info(f"{p2}")
        # # p2 = start_node._nextNode._xy + start_node._xy
        # steering_angle = TrackHelpers.getAngleRotation(np.array(p1), np.array(p2))
        # steering_angle = np.rad2deg(steering_angle)  # convert to degrees


        # publish msgs
        # self.steering_angle.publish(Float32(data=steering_angle))
        p3 = start_node._stateList[0]._nextState._nextState._xy
        points = [p1, p2, p3]
        msg = PoseArray()
        for P in points:
            args = {"position": Point(x=P[0], y=P[1], z=0.0)}
            msg.poses.append(Pose(**args))
        self.best_trajectory_publisher.publish(msg)

        # self.get_logger().info(f"steering angle published: {steering_angle}")

        return
    
    def getStartingPosition(self, car_position, brackets):
        best_dist = np.Inf
        best_bracket_idx = 0
        for i, B in enumerate(brackets):
            dists = [TrackHelpers.getDistance(car_position, node._xy) for node in B._nodeList]
            if min(dists) < best_dist:
                best_dist = min(dists)
                best_bracket_idx = i
                starting_point = brackets[i]._nodeList[np.argmin(dists)]._xy
        # delete brackets before the starting position
        # brackets = brackets[best_bracket_idx:best_bracket_idx+5]
        brackets = brackets[best_bracket_idx:]

        return starting_point, brackets

    
    def get_boundaries(self, cones):
        # loop through each cone
        innerboundary = []
        outerboundary = []
        ic = 0
        oc = 0
        for i in range(len(cones)):
            x = cones[i].pose.pose.position.x
            y = cones[i].pose.pose.position.y
            if i == 0:
                car_position = [x,y]
                w = cones[i].pose.pose.orientation.w
            else:
                # blue - left
                if cones[i].colour == 0:
                    innerboundary.append([x,y])
                    ic += 1
                elif cones[i].colour == 2:
                    outerboundary.append([x,y])
                    oc += 1

        return innerboundary, outerboundary, car_position, w, oc, ic
    
    def save_track(self, innerboundary, outerboundary):
        with open(f'/{os.path.dirname(__file__)}/bound_coods', 'w') as fh:
                xl=[i[0] for i in innerboundary]
                yl=[i[1] for i in innerboundary]
                xr=[i[0] for i in outerboundary]
                yr=[i[1] for i in outerboundary]
                for P in xl:
                    fh.write("{} ".format(P))
                fh.write("\n")
                for P in yl:
                    fh.write("{} ".format(P))
                fh.write("\n")
                for P in xr:
                    fh.write("{} ".format(P))
                fh.write("\n")
                for P in yr:
                    fh.write("{} ".format(P))
                fh.write("\n")
                fh.close()
    

    def create_track_dataframe(self, innerboundary:np.array, outerboundary:np.array):
        # up_to = min(len(innerboundary), len(outerboundary)) 
        return pd.DataFrame({
            "inner": list(innerboundary),
            "outer": list(outerboundary),
        })


    def get_center_line(self, innerboundary:np.array, outerboundary:np.array):
        """computes the center line and track width to left and right boundaries

            return:
                center line: list of lists of coorindates as [x,y]
                track widths: list of list of widths as [to_left, to_right]
        """
        constant_width = TrackHelpers.getDistance(p1=innerboundary[0], p2=outerboundary[0]) 
        constant_width = constant_width 

        # get center points
        n_points = min(len(innerboundary), len(outerboundary)) #unfortunately not always same number of points
        center_points = [0]*n_points
        track_widths = [0]*n_points

        for i in range(n_points):
            pI = innerboundary[i]
            pO = outerboundary[i]
            # get center point
            center_points[i] = TrackHelpers.getMidPoint(pI,pO)
            track_widths[i] = [constant_width, constant_width]

        return center_points, track_widths
    

    def get_track_info(self, center_line:np.array, track_widths:np.array):
        """creates a pandas dataframe for the center line coordiantes and track widths"""
        return pd.DataFrame({
            "x_m": [P[0] for P in center_line],
            "y_m": [P[1] for P in center_line],
            "w_tr_left_m": [P[0] for P in track_widths],
            "w_tr_right_m": [P[1] for P in track_widths],
        })
    

    def get_start_node(self, starting_point): 
        return Node(1, np.array(starting_point), None, None)
    

    def get_transformed_point(self, cone_map:ConeMap, point:np.array):
        position_and_orientation = self.get_position_of_cart(cone_map)
        position_vector, rotation_matrix = self.get_transformation_matrix(position_and_orientation)
        pre_trans_points = self.apply_transformation(position_vector, rotation_matrix, point[0], point[1])

        return [pre_trans_points[0][0], pre_trans_points[1][0]]

    
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
        transformed_point = np.matmul(rotation_matrix, point) + position_vector

        return transformed_point


def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    node = shortest_path()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()