#!/usr/bin/env python3
# Python imports
from typing import Optional
import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from geometry_msgs.msg import PoseArray
from moa_msg.msg import ConeMap
from ackermann_msgs.msg import AckermannDrive



class path_planning(Node):
    def __init__(self):
        super().__init__("Pure_Pursuit_Controller")
        self.get_logger().info("Path Planning Node Started")
        
        self.steering_angle = 0
        self.look_ahead = 100
        self.pos = (0,0)

        # subscribe to best trajectory
        self.best_trajectory_sub = self.create_subscription(PoseArray, "moa/selected_trajectory", self.selected_trajectory_handler, 5)
        self.cone_map_sub = self.create_subscription(PoseArray, "moa/selected_trajectory", self.cone_map_handler, 5)
        self.cmd_vel_pub = self.create_publisher(AckermannDrive, "cmd/vel", 5)

    def cone_map_handler(msg: ConeMap):
        self.car_pose = msg.cones[0].pose.position

    def selected_trajectory_handler():
        self.set_steering_angle(self.curvator())

    def publish_ackermann():
        
        args = {"steering_angle": self.steering_angle,
                "steering_angle_velocity": 0.0,
                "speed": self.current_speed,
                "acceleration": 0.0,
                "jerk": 0.0}
        msg = AckermannDrive(**args)
        self.cmd_vel_pub.publish(msg)

    def lateral_distance

    def arc_radius():
        y = self.lateral_distance(self.way_point)
        pow(self.look_ahead, 2)  / 2 * y

    def curvator():
        self.look_ahead

    def set_steering_angle():
        steering_angle = curvator