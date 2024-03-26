#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PoseArray
from moa_msgs.msg import ConeMap

import numpy as np

class trajectory_following(Node):
    def __init__(self):
        super().__init__("Trajectory_Following")
        self.get_logger().info("Trajectory Following Node Started")

        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('debug', False)
        #     ]
        # )
        self._distance_to_front = 2.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.declare_parameter('car_name', 'test')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value
        steering_topic = "/" + car_name + "/cmd_steering"
        torque_topic = "/" + car_name + "/cmd_throttle"
        # speed_topic = "/" + car_name + "/speed"
        
        # subscribers
        self.create_subscription(PoseArray, "moa/selected_trajectory", self.get_desired_pose, qos_profile)
        self.create_subscription(Float32, "moa/selected_steering_angle", self.get_steering_angle, qos_profile)  
        self.create_subscription(ConeMap, "cone_map", self.callback, qos_profile)

        # publishers (including simulation)
        self.moa_steering_pub = self.create_publisher(AckermannDrive, "cmd_vel", 10)
        self.sim_steering_pub = self.create_publisher(Float32, steering_topic, 10)
        # self.feedback_subscribe = self.create_subscription(Float64,speed_topic,self.set_speed,10)
    
    def get_steering_angle(self, msg:Float32): self._steering_angle = msg.data  # radians
    
    def get_desired_pose(self, msg:PoseArray): self._desired_pose = msg.poses[-1]

    # def get_car_position(self, msg:ConeMap): self._car_position = msg.cones[0].pose.pose.position

    def callback(self, msg:ConeMap):
        if hasattr(self, "_steering_angle") and hasattr(self, "_desired_pose"):
            # get current and desired points
            car_position = msg.cones[0].pose.pose.position
            current = np.array([car_position.x, car_position.y + self._distance_to_front])
            desired = np.array([self._desired_pose.position.x, self._desired_pose.position.y])
            
            # compute the distance between the desired and current point
            error = self.get_control_error(current, desired)
            # get p-gain 
            p_gain = self.get_gain(error)

            # compute new angle in degrees
            steering_angle_rad = p_gain * self._steering_angle 

            self.get_logger().info(f"before and after gain: {steering_angle_rad/p_gain}, {steering_angle_rad}")
            
            steering_angle_deg = self._steering_angle * 180 / np.pi

            # publish msgs
            args = {"steering_angle": float(steering_angle_deg),
                    "steering_angle_velocity": 0.0,
                    "speed": 0.0,
                    "acceleration": 0.0,
                    "jerk": 0.0}

            self.moa_steering_pub.publish(AckermannDrive(**args))
            self.sim_steering_pub.publish(Float32(data=steering_angle_deg))

            return
        
        self.get_logger().info("waiting for moa/selected_trjectory topic")

        return
    
    
    def get_control_error(self, current:np.array, desired:np.array): return np.linalg.norm(current-desired, ord=2)

    def get_gain(self, distance):
        """return gain on steering angle based on error - distance currently"""
        return (distance-0)/(4-0)




def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    node = trajectory_following()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()