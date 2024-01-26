#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from random import random, randint, uniform

class PublishAckermannMsg(Node):
    def __init__(self):
        super().__init__("ackermann_publiser_node")
        self.ackerman_publisher = self.create_publisher(AckermannDriveStamped, "cmd_vel", 10)
        self.create_timer(10, self.publish_msg)
    
    def publish_msg(self):
        args = {"steering_angle": uniform(-0.785,0.785),
                "steering_angle_velocity": random(),
                "speed": float(randint(0,30)),
                "acceleration": float(randint(0,255)),
                "jerk": random()}
        
        # args = {"steering_angle": 0.5,
        #         "steering_angle_velocity": 0.5,
        #         "speed": 1.0,
        #         "acceleration": 0.0,
        #         "jerk": 0.0}

        args2 = {'stamp':Time(sec=1,nanosec=2),
                 'frame_id':'ack_to_can_test'}
        
        args3 = {'header': Header(**args2),
                 'drive':AckermannDrive(**args)}
        
        ackermsg = AckermannDriveStamped(**args3)
        print(ackermsg)
        self.ackerman_publisher.publish(ackermsg)


def main():
    rclpy.init()

    node = PublishAckermannMsg()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()