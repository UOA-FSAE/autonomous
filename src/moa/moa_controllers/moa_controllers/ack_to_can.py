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
import struct

def float_to_bin(f):
    hex_str = hex(struct.unpack('<I', struct.pack('<f', f))[0])
    bin_str = bin(int(hex_str, 16))[2:].zfill(32)
    return bin_str

def twos_comp(num, no_bits):
    #input is float32 for ackermann
    if -(2**(no_bits-1)) <= num <= (2**(no_bits-1)-1):
        return ~num #doesnt work for floats

def ackermann_to_can_parser(ack_msg: AckermannDriveStamped) -> CANStamped:
        can_msg = CANStamped 
        
        #unpack received Ackermann msg
        can_msg.can.id = 56 #TODO need to change to dynamic
        can_msg.can.data = [     
            #if use c_uint8 it overflows e.g. 257 becomes 1
        
            ack_msg.drive.speed, #need to check between 0-256
            ack_msg.drive.acceleration, #need to check between 0-256
            ack_msg.drive.jerk, #need to check between 0-256
            ack_msg.drive.steering_angle, #can be negative
            ack_msg.drive.steering_angle_velocity, #can be negative
            0,
            0,
            0
        ]

        return can_msg


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

    def ack_to_can_publish_callback(self, ack_msg: AckermannDriveStamped):
        can_msg: CANStamped = ackermann_to_can_parser(ack_msg)
        self.can_pub.publish(can_msg)
        
    
def main(args=None):
    rclpy.init(args=args)

    node = ack_to_can()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
