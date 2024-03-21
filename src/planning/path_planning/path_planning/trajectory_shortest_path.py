#!/usr/bin/python3
from path_planning.shortest_path.CoreModels import Node 
import path_planning.shortest_path.TrackMethods as TrackMethods
import path_planning.shortest_path.TrackHelpers as TrackHelpers

import numpy as np
import pandas as pd
# from shapely import LineString, MultiPoint
# from shapely import Point as shapelyPoint
import os

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
                ('plot', False)
            ]
        )

        # attributes
        self._plot = self.get_parameter("plot").get_parameter_value().bool_value

        # subscribers
        self.create_subscription(ConeMap, "cone_map", self.set_boundaries, 10)

        # publishers
        self.steering_angle = self.create_publisher(Float32, "moa/selected_steering_angle", 10)
        self.best_trajectory_publisher = self.create_publisher(PoseArray, "moa/selected_trajectory", 10)


    def set_boundaries(self, msg:ConeMap):
        cones = msg.cones
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
            else:
                # blue - left
                if cones[i].colour == 0:
                    innerboundary.append([x,y])
                    ic += 1
                elif cones[i].colour == 2:
                    outerboundary.append([x,y])
                    oc += 1

        # innerboundary = np.array(innerboundary)
        # outerboundary = np.array(outerboundary)
        innerboundary = np.array(sorted(innerboundary))
        outerboundary = np.array(sorted(outerboundary))
        # firstO = outerboundary[0]
        # lastO = outerboundary[-1]
        # outerboundary[0] = innerboundary[0]
        # outerboundary[-1] = innerboundary[-1]
        # innerboundary[0] = firstO
        # innerboundary[-1] = lastO
        if ic > oc: innerboundary = innerboundary[:(oc-ic)] 
        if oc > ic: outerboundary = outerboundary[:(ic-oc)]
    
        # get center line and track widths
        center_line, track_widths = self.get_center_line(innerboundary, outerboundary)
        # format as pandas data frame
        track_info = self.get_track_info(center_line, track_widths)

        # get shortest path
        print("IMPORTING TRACK")
        # df = TrackMethods.importTrack(track_info=track_info, plot=self._plot)

        df = self.create_track_dataframe(list(innerboundary), list(outerboundary))
        self.save_track(df.inner, df.outer)

        # create brackets
        print("CREATING BRACKETS")
        brackets = TrackMethods.getBrackets(df, 10, plot=self._plot)

        velocity_range = [0.01, 8, 16, 24, 32, 40]# velocities in meters per second

        # compute optimal path
        print("COMPUTING OPTIMAL PATH")
        start_node = self.get_start_node(starting_point=car_position)
        print("starting inner distance: ", start_node._innerDistance)
        print("starting outer distance: ", start_node._outerDistance)
        start_node = TrackMethods.belman_ford_path(df, velocity_range, brackets, start_node, plot=self._plot)
        print("\nOPTIMAL PATH COMPUTED")

        # get steering angle based on current and next point
        p1 = start_node._xy
        p2 = start_node._nextNode._xy
        # p1 = np.array(self.get_transformed_point(msg, start_node._xy))
        p2 = np.array(self.get_transformed_point(msg, start_node._nextNode._xy))
        steering_angle = TrackHelpers.getAngle(p1=p1, p2=p2)

        # publish msgs
        self.steering_angle.publish(Float32(data=steering_angle))

        nodes = [start_node, start_node._nextNode]
        msg = PoseArray()
        for N in nodes:
            args = {"position": Point(x=N._xy[0], y=N._xy[1], z=0.0)}
            msg.poses.append(Pose(**args))
        self.best_trajectory_publisher.publish(msg)

        self.get_logger().info(f"steering angle published: {steering_angle}")

        return
    
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
    

    def create_track_dataframe(self, innerboundary:list, outerboundary:list):
        return pd.DataFrame({
            "inner": innerboundary,
            "outer": outerboundary,
        })


    def get_center_line(self, innerboundary:np.array, outerboundary:np.array):
        """computes the center line and track width to left and right boundaries

            return:
                center line: list of lists of coorindates as [x,y]
                track widths: list of list of widths as [to_left, to_right]
        """
        constant_width = TrackHelpers.getDistance(p1=innerboundary[0], p2=outerboundary[0]) 
        constant_width = constant_width / 3

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
        return Node(-1, np.array(starting_point), 0, None, None, np.nan, 0)
    

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