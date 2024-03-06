#!/usr/bin/python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose, PoseArray
from moa_msgs.msg import ConeMap, AllStates, AllTrajectories
from ackermann_msgs.msg import AckermannDrive


class trajectory_generator(Node):
    def __init__(self):
        super().__init__("trajectory_generation")
        self.get_logger().info("Trajectory generation Node Started")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug', False),
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
            paths, states = self.trajectory_generator(self._cone_map)
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
    # TRAJECTORY GENERATION
    def single_trajectory_generator(self, steering_angle, position_vector, rotation_matrix):
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357 is used for
        # bicycle steering
        L = 1;
        R = L / np.tan(steering_angle);
        # list of time in space
        t_range = np.arange(0, np.pi/6, 0.008);
        trajectory_output = PoseArray();
        for i, individual_t in enumerate(t_range):
            # shorten trajectory lengths
            # if i > 25:
            #     break
            # pose for current time in space
            pose_input = Pose();
            # pre transformation coordinates
            x_pre_trans = np.cos(individual_t) * R - R
            y_pre_trans = abs(np.sin(individual_t) * R)
            # post transformation coordinates (relative to car)
            post_trans_point = self.apply_transformation(position_vector, rotation_matrix, x_pre_trans, y_pre_trans);
            pose_input.position.x = post_trans_point[0][0]
            pose_input.position.y = post_trans_point[1][0]
            trajectory_output.poses.append(pose_input)
        return trajectory_output

    def trajectory_generator(self, cone_map):
        # steering angles
        candidate_steering_angle = np.deg2rad(np.arange(-10, 10, 0.1))
        trajectories = []
        # get position of car (first cone in cone map data)
        position_and_orientation = self.get_position_of_cart(cone_map)
        # get transformation matrix 
        position_vector, rotation_matrix = self.get_transformation_matrix(position_and_orientation)
        for steering_angle in candidate_steering_angle:
            if steering_angle == 0:
                steering_angle = 1e-6;
            # generate trajectory for this angle
            added_trajectory = self.single_trajectory_generator(steering_angle, position_vector, rotation_matrix)
            trajectories.append(added_trajectory)

        # # straight trajectory
        # candidate_steering_angle = np.append(candidate_steering_angle, 0)
        # trajectories.append(self.get_straight_trajectory(cone_map))
        return trajectories, candidate_steering_angle
    
    def get_straight_trajectory(self, cone_map):
        first_cone = cone_map.cones[0].pose.pose
        posearray = PoseArray()
        # number of points in y direction
        num_y = 158
        # num spacing (m?)
        spacing = 2.0
        # starting point
        x_start = first_cone.position.x
        y_start = first_cone.position.y
        for i in range(num_y):
            pose = Pose()
            y_point = (i+1)*spacing
            position_and_orientation = self.get_position_of_cart(cone_map)
            # get transformation matrix 
            position_vector, rotation_matrix = self.get_transformation_matrix(position_and_orientation)
            x_pre_trans = x_start
            y_pre_trans = y_start + y_point
            out = self.apply_transformation(position_vector, rotation_matrix, x_pre_trans, y_pre_trans)
            pose.position.x = out[0][0]
            pose.position.y = out[1][0]
            # append to pose list
            posearray.poses.append(pose)

        return posearray

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



