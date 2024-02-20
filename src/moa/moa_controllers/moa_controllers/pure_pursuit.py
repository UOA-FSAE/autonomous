#!/usr/bin/env python3
# Python imports
from typing import Optional
import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from math import sqrt

from geometry_msgs.msg import PoseArray, Pose
from moa_msg.msg import ConeMap, Cone
from ackermann_msgs.msg import AckermannDrive



class PurePursuitController(Node):
    def __init__(self):
        super().__init__("Pure_Pursuit_Controller")
        self.get_logger().info("Path Planning Node Started")
        
        self.steering_angle = 0
        self.look_ahead = 100
        self.pos = None
        self.way_point = None
        # subscribe to best trajectory
        self.best_trajectory_sub = self.create_subscription(PoseArray, "moa/selected_trajectory", self.selected_trajectory_handler, 5)
        self.cmd_vel_pub = self.create_publisher(AckermannDrive, "cmd/vel", 5)

        self.timer = self.create_timer(5, self.publish_ackermann)

    def selected_trajectory_handler(self, poseArray: PoseArray) -> None:
        # The first marker location is the car 
        # TODO separate into its own topic
        self.car_pose = poseArray[0].position
        # The cars orientation is stored as radians from the car's initial
        # orientation (following right hand rule for positive and negative
        # direction) as the manitude of the quaternion (w). 
        self.car_orientation = poseArray[0].orientation.w

        cones = poseArray[1:]
        self.way_point = self.way_point_look_ahead(cones)

        
        self.set_steering_angle(self.curvator())

        self.cmd_vel_pub.publish()

    def extract_cones(self, coneMap: ConeMap) -> list[Cone]:
        return coneMap[1:]
    
    def way_point_look_ahead(self, cones: ConeMap) -> Pose:
        way_point = cones[0].pose
        distance = float('inf')
        for cone in cones:
            distance_from_look_ahead_radius = self.distance_from_look_ahead_radius(cone.pose.position)
            if distance_from_look_ahead_radius < distance:
                way_point = cone.pose
                distance = distance_from_look_ahead_radius
        return way_point

    def distance_from_look_ahead_radius(self, pos):
        return abs(self.distanceTo(pos) - self.look_ahead)

    def distanceTo(self, pos):
        return sqrt(pow((self.pose.postion.x - pos.x),2) + pow((self.pose.postion.y - pos.y),2))

    def publish_ackermann(self) -> None:
        
        if (self.steering_angle == None): return

        args = {"steering_angle": self.steering_angle,
                "steering_angle_velocity": 0.0,
                "speed": self.current_speed,
                "acceleration": 0.0,
                "jerk": 0.0}
        msg = AckermannDrive(**args)
        self.cmd_vel_pub.publish(msg)

    def lateral_distance()
        

    def arc_radius(self):
        y = self.lateral_distance(self.way_point)
        pow(self.look_ahead, 2)  / 2 * y

    def curvator(self):
        return 1/self.arc_radius()

    def set_steering_angle(self, angle):
        self.steering_angle = angle



def main():
    rclpy.init()
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"node spin error: {e}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()