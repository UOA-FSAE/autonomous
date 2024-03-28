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
        if hasattr(self, "_steering_angle"):
            # get current and desired points
            car_position = msg.cones[0].pose.pose.position
            current = np.array([car_position.x, car_position.y])

            # boundaries
            innerboundary, outerboundary = self.get_boundaries(msg.cones)
            desired = self.get_center_points(innerboundary, outerboundary)
            
            # compute the distance between the desired and current point
            error = self.get_control_error(current, desired)
            # get p-gain 
            track_width = self.get_track_width(innerboundary, outerboundary)
            p_gain = self.get_gain(min(error), track_width)
            

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
    
    
    def get_control_error(self, current:np.array, desired:np.array): return np.linalg.norm(current-desired, axis=1)

    def get_gain(self, distance, track_width):
        """return gain on steering angle based on error - distance to center line currently"""
        self.get_logger().info(f"DISTANCE = {distance}")
        center_to_boundary_distance = track_width

        return (distance/center_to_boundary_distance)**2
    
    def get_center_points(self, innerboundary, outerboundary):
        """get the center points"""
        up_to = min(len(innerboundary), len(outerboundary))
        center_points = []
        for i in range(up_to):
            x1, y1 = innerboundary[i]
            x2, y2 = outerboundary[i]
            x, y = self.get_average_point(x1,y1,x2,y2)
            center_points.append([x,y])

        return center_points
    
    def get_track_width(self, innerboundary, outerboundary):
        firstInner = np.array(innerboundary[0])
        firstOuter = np.array(outerboundary[0])
        width = self.get_distance(firstInner, firstOuter)

        return width
    
    def get_boundaries(self, cones):
        innerboundary = []
        outerboundary = []
        for i in range(len(cones)):
            if i != 0:
                x = cones[i].pose.pose.position.x
                y = cones[i].pose.pose.position.y
                # blue - inner
                if cones[i].colour == 0:
                    innerboundary.append([x,y])
                elif cones[i].colour == 2:
                    outerboundary.append([x,y])
        
        return innerboundary, outerboundary

    def get_average_point(self, x1,y1,x2,y2): return (x1+x2)/2, (y1+y2)/2

    def get_distance(self, p1:np.array, p2:np.array): return np.sqrt(sum((p2-p1)**2))



def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    node = trajectory_following()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()