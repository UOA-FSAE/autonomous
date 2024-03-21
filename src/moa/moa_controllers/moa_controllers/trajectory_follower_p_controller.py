#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32, Float64
import numpy as np

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
        # self.p_controlled_pub = self.create_publisher(AckermannDrive, "cmd_vel", 5)
        # subscribe to best trajectory
        # self.best_traj_sub = self.create_subscription(PoseArray, "moa/selected_trajectory", self.get_best_state, 5)
        self.create_subscription(Float32, "moa/selected_steering_angle", self.get_best_state, 10)
        # self.current_states_sub = self.create_subscription(AckermannDrive, "moa/cur_vel", self.get_current_states, 5)
        

        self.declare_parameter('car_name', 'test')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value
        steering_topic = "/" + car_name + "/cmd_steering"
        torque_topic = "/" + car_name + "/cmd_throttle"
        speed_topic = "/" + car_name + "/speed"
        self.steering_publisher = self.create_publisher(Float32, steering_topic, 10)
        # self.torque_publisher = self.create_publisher(Float32, torque_topic, 10)
        self.feedback_subscribe = self.create_subscription(Float64,speed_topic,self.set_speed,10)
    
    def set_speed(self, msg:Float64):
        self.current_speed = msg.data

    def get_current_states(self, msg: AckermannDrive) -> None: 
        self.current_speed = msg.speed
        self.current_angle = msg.steering_angle

    def get_control_error(self, csa, dsa): return dsa-csa

    def get_best_state(self, msg: Float32):
        p_gain = 1
        steering_angle_deg = (msg.data*180/np.pi) * p_gain
        # if steering_angle_deg < -9.0:
        #     steering_angle_deg = -30.0
        # elif steering_angle_deg > 9.0:
        #     steering_angle_deg = 30.0
        self.get_logger().info(f"before and after gain: {steering_angle_deg/p_gain}, {steering_angle_deg}")
        self.steering_publisher.publish(Float32(data=steering_angle_deg))
        # self.get_logger().info(f"chosen recieved state is = {msg.steering_angle}")

        # if self.debug:
        #     self.current_angle = self.current_speed = 0.0

        # if hasattr(self,"current_speed") and hasattr(self,"current_angle"):
        #     # error 
        #     error = self.get_control_error(self.current_angle, msg.steering_angle)
            
        #     # constant gain multiplier
        #     p_gain = 0.5

        #     # new steering angle output
        #     chosen_state = self.current_angle + error * p_gain

        #     # publish msg to /moa/drive
        #     args = {"steering_angle": float(chosen_state),
        #             "steering_angle_velocity": 0.0,
        #             "speed": self.current_speed,
        #             "acceleration": 0.0,
        #             "jerk": 0.0}
            
        #     msg = AckermannDrive(**args)
        #     self.p_controlled_pub.publish(msg)
        #     self.get_logger().info(f"P-controlled state published = {chosen_state}")

        #     return
        
        # self.get_logger().info("Attributes current speed/angle not initialised")
        # return

def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    node = trajectory_following()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()