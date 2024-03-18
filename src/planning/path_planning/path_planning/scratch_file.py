#!/usr/bin/python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Point, Pose, PoseArray
from moa_msgs.msg import ConeMap, AllStates, AllTrajectories
from ackermann_msgs.msg import AckermannDrive

import numpy as np
import matplotlib.pyplot as plt


class trajectory_generator(Node):
    def __init__(self):
        super().__init__("trajectory_generation")
        self.get_logger().info("Trajectory generation Node Started")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug', True),
                ('timer', 3.0),
            ]
        )

        # attributes
        self._id = 0
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self._timer = self.get_parameter('timer').get_parameter_value().double_value

        # publishers
        self.all_trajectories_publisher = self.create_publisher(AllTrajectories, "moa/trajectories", 10)
        self.all_states_publisher = self.create_publisher(AllStates, "moa/states", 10)

        # subscribers
        self.create_subscription(AckermannDrive, "moa/cur_vel", self.set_current_speed, 10)
        self.create_subscription(ConeMap, "cone_map", self.set_cone_map, 10)

        # time in between trajectory generation
        self.create_timer(self._timer, self.generate_trajectories)


    def set_current_speed(self, msg:AckermannDrive) -> None: self._current_speed = msg.speed
    
    def set_cone_map(self, msg:ConeMap) -> None: self._cone_map = msg


    # BEST TRAJECTORY PUBLISHER
    def generate_trajectories(self):
        '''Generates trajectories every few seconds'''

        if self._debug: self._current_speed = 0.0

        self.get_logger().info(f"{self._timer} seconds up - generating trajectories")
        self.get_logger().info(f"current speed: {hasattr(self,'_current_speed')}"\
                               f" | cone map: {hasattr(self,'_cone_map')}")

        if hasattr(self,"_current_speed") and hasattr(self,"_cone_map"):
            # generate trajectories
            paths, states = self.my_trajectory_generator(cone_map=self._cone_map, radius=2.5, npoints=100)

            # publish states and trajectories
            state_list = []
            for i in range(len(states)):
                args = {"steering_angle": float(states[i]),
                        "steering_angle_velocity": 0.0,
                        "speed": self._current_speed,
                        "acceleration": 0.0,
                        "jerk": 0.0}
                state_list.append(AckermannDrive(**args))
            
            msg = AllStates(id = self._id, states = state_list)
            self.all_states_publisher.publish(msg)

            msg = AllTrajectories(id = self._id, trajectories = paths)
            self.all_trajectories_publisher.publish(msg)

            self.get_logger().info("msg published")

            self._id += 1

            return

        return

    # ===========================================================
    # # TRAJECTORY GENERATION
    def my_trajectory_generator(self, cone_map, radius, npoints):
        """generate straight trajectory based on given radius from origin (0,0)"""
        trajectories = []
        # 1. initial point
        first_cone = cone_map.cones[0].pose.pose.position
        # cor = [first_cone.x, first_cone.y]
        cor = [0,0]
        # 2. radius
        r = radius
        # 3. x points 
        n = npoints
        x = np.linspace(-r/2, r/2, n, endpoint=False)[1:]
        n -= 1
        y = np.zeros(n)
        angs = np.zeros(n)

        for i, val in enumerate(x):
            tmp_path = PoseArray()
            # append starting point
            tmp_pose = Pose()
            tmp_pose.position.x = first_cone.x
            tmp_pose.position.y = first_cone.y
            tmp_path.poses.append(tmp_pose)

            # 4. find y using equation of circle
            y[i] = np.sqrt(np.round(r**2-(val-cor[0])**2, 3)) + cor[1]
            # 5. calculate steering angle for each trajectory
            dy = y[i] - cor[1]
            dx = val - cor[0]
            # inverse tan is in radians
            angle = np.arctan(dx/dy)
            angs[i] = (angle if dx < 0 else angle)
            print(f"passed angle = {angs[i]} for dx = {dx}")
            
            # transform coordinates from fixed to car 
            car_position = self.get_position_of_cart(cone_map)
            car_position, rotation_matrix = self.get_transformation_matrix(car_position)
            post_trans_pose = self.apply_transformation(car_position, rotation_matrix, val, y[i])
            # append new points to pose array
            x[i] = post_trans_pose[0][0]
            y[i] = post_trans_pose[1][0]
            tmp_pose2 = Pose()
            tmp_pose2.position.x = x[i]
            tmp_pose2.position.y = y[i]
            tmp_path.poses.append(tmp_pose2)

            if x[i] is np.nan or y[i] is np.nan or angs[i] is np.nan:
                print("NAN!")

            # plotting
            # plt.plot([cor[0],x[i]],[cor[1],y[i]],label=f'trajectories')

            # append trajecotry
            trajectories.append(tmp_path)

        # plt.show()

        # list of points as tuples
        points = [(x[i],y[i]) for i in range(n)]

        return trajectories, angs

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
    node = trajectory_generator()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
