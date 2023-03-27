#/usr/bin/env python3
from typing import Optional
import rclpy
from rclpy.node import Node
import time

from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from moa_msgs.msg import CANStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import numpy as np
# import zlib
from math import pi


# def compress_floats(values:np.ndarray) -> bytes:    
#     # Compute the differences between consecutive values
#     differences = np.diff(values)

#     # Compress the differences using zlib
#     compressed = zlib.compress(differences, level=9)

#     return compressed


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

    def ackermann_to_can_parser(self, ack_msg: AckermannDriveStamped) -> Optional[CANStamped]: #type for option optionalnull
        #checks before sending Ackermann
        if 0 < ack_msg.drive.speed > 120/3.6: #m/s   
            self.get_logger().warn(f'ackermann drive SPEED out of bounds')
            return None
        
        elif 0 < ack_msg.drive.acceleration > 256: #m/s^2
            self.get_logger().warn(f'ackermann drive ACCLERATION out of bounds')
            return None

        elif 0 < ack_msg.drive.jerk: #m/s^3 
            #not too fussed about assign 1 byte (minifloat)
            self.get_logger().warn(f'ackermann drive JERK out of bounds')
            return None
        
        elif -45*pi/180 < ack_msg.drive.steering_angle > 45*pi/180: #radians
            self.get_logger().warn(f'ackermann drive STEERING_ANGLE out of bounds')
            return None
        
        elif 0 < ack_msg.drive.steering_angle_velocity < 1: #radians/s
            #unsure of upper limit def dont need more than 1
            self.get_logger().warn(f'ackermann drive STEERING_ANGLE_VELOCITY out of bounds')
            return None
        
        #format values of Ackermann
        speed = np.uint8(round(ack_msg.drive.speed)) #(functionally limited to 100-120km/h) integer and round 
        acceleration = np.uint8(round(ack_msg.drive.acceleration))
        jerk = np.uint8(ack_msg.drive.jerk*1000)
        steering_angle = np.float16(ack_msg.drive.steering_angle)
        steering_angle_vel = np.uint8(ack_msg.drive.steering_angle_velocity*1000)
        
        #separator for steering_angle
        sav_size = len(steering_angle_vel)

        #Compose CAN data packet
        ackermann_vals = [
            speed.tobytes(),
            acceleration.tobytes(), 
            jerk.tobytes(), 
            steering_angle.tobytes()[:sav_size//2], 
            steering_angle.tobytes()[sav_size//2:],
            steering_angle_vel.tobytes(), 
            ]

        # #compress Ackermann values
        # compressed_ack_vals = compress_floats(ackermann_vals)
        
        # #separate compressed ackermann values to list of bytes for CAN
        # data_packets = [compressed_ack_vals[i:i+1] for i in range(0, len(compressed_ack_vals), 1)]

        return ackermann_vals


    def ack_to_can_publish_callback(self, ack_msg: AckermannDriveStamped):
        can_msg = CANStamped 

        #configure header
        can_header = Header()
        can_header.stamp = self.get_clock().now()
        can_header.frame_id = 'ackermann_to_can'

        #set CAN header/data/id 
        can_msg.header = can_header
        can_msg.can.id = 56 #TODO need to change to dynamic
        can_msg.can.data = self.ackermann_to_can_parser(ack_msg)

        if can_msg.can.data is not None:
            #publish CAN to topic
            self.can_pub.publish(can_msg)
        
        return 
    

def main(args=None):
    rclpy.init(args=args)

    node = ack_to_can()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
