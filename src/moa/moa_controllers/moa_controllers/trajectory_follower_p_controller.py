#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

class trajectory_following(Node):
    def __init__(self):
        super().__init__("Trajectory_Following")
        self.get_logger().info("Trajectory Following Node Started")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug', False)
            ]
        )

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # publish p-controlled trajectory
        self.p_controlled_pub = self.create_publisher(AckermannDrive, "cmd_vel", 5)
        # subscribe to best trajectory
        self.best_traj_sub = self.create_subscription(AckermannDrive, "moa/selected_trajectory", self.get_best_state, 5)
        self.current_states_sub = self.create_subscription(AckermannDrive, "moa/cur_vel", self.get_current_states, 5)

    def get_current_states(self, msg: AckermannDrive) -> None: 
        self.current_speed = msg.speed
        self.current_angle = msg.steering_angle

    def get_control_error(self, csa, dsa): return dsa-csa

    def get_best_state(self, msg: AckermannDrive):
        self.get_logger().info(f"chosen recieved state is = {msg.steering_angle}")

        if self.debug:
            self.current_angle = self.current_speed = 0.0

        if hasattr(self,"current_speed") and hasattr(self,"current_angle"):
            # error 
            error = self.get_control_error(self.current_angle, msg.steering_angle)
            
            # constant gain multiplier
            p_gain = 0.5

            # new steering angle output
            chosen_state = self.current_angle + error * p_gain

            # publish msg to /moa/drive
            args = {"steering_angle": float(chosen_state),
                    "steering_angle_velocity": 0.0,
                    "speed": self.current_speed,
                    "acceleration": 0.0,
                    "jerk": 0.0}
            
            msg = AckermannDrive(**args)
            self.p_controlled_pub.publish(msg)
            self.get_logger().info(f"P-controlled state published = {chosen_state}")

            return
        
        self.get_logger().info("Attributes current speed/angle not initialised")
        return

