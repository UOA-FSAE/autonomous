#/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ctypes import c_uint8
import time

from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from moa_msgs.msg import CANStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class ack_to_can(Node):
    def __init__(self):
        super().__init__('ackermann_to_can') #node name (NB: MoTec listens to this)

        #create subscriber for ackermann input
        self.subscription = self.create_subscription(
            AckermannDriveStamped,     #msg type
            'cmd_val',                 #topic receiving from
            self.listener_callback,    #callback function
            10                         #qos profile
        )

        #create server for CAN
        self.can_pub = self.create_publisher( #Unsure if needs to be server or client
            CANStamped,
            'pub_raw_can',
            10  
        )

    def listener_callback(self, msg): #TODO
        can_msg = CANStamped 
        
        #unpack received Ackermann msg
        can_msg.can.id = 56
        can_msg.can.data = [
            msg.drive.speed,
            msg.drive.acceleration,
            msg.drive.jerk,
            msg.drive.steering_angle,
            msg.drive.steering_angle_velocity,
            0,
            0,
            0
        ]

        self.get_logger().info(msg.data)

        #sends CAN to topic
        self.can_pub.publish(can_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ack_to_can()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
