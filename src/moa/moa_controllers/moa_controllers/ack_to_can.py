#!/usr/bin/env python3
# Python imports
from typing import Optional
import rclpy
from rclpy.node import Node
import numpy as np
# import zlib
from math import pi

# Ros Imports
from ackermann_msgs.msg import AckermannDriveStamped
from moa_msgs.msg import CANStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


# def compress_floats(values:np.ndarray) -> bytes:    
#     # Compute the differences between consecutive values
#     differences = np.diff(values)

#     # Compress the differences using zlib
#     compressed = zlib.compress(differences, level=9)

#     return compressed


class ack_to_can(Node):
    def __init__(self):
        super().__init__('ackermann_to_can') # node name (NB: MoTec listens to this)

        # create subscriber for ackermann input
        self.subscription = self.create_subscription(
            AckermannDriveStamped,     # msg type
            'cmd_val',                 # topic receiving from
            self.ack_to_can_publish_callback,    #callback function
            10                         # qos profile
        )

        # create publisher for CAN
        self.can_pub = self.create_publisher( 
            CANStamped,
            'pub_raw_can',
            10  
        )

    def ackermann_to_can_parser(self, ack_msg: AckermannDriveStamped) -> Optional[CANStamped]:
        """
        Parses an AckermannDriveStamped message into a list of CAN data bytes.

        Args
        ---------------------------
        - ack_msg: A ROS message of type `AckermannDriveStamped`, containing the Ackermann drive commands to be parsed.

        Returns
        ---------------------------
        - A list of integers representing the parsed Ackermann drive values, suitable for sending over the CAN bus.
        Returns `None` if any of the Ackermann values are out of bounds or invalid.

        The following Ackermann drive values are checked for validity:
        - `speed`: Must be between 0 and 120 km/h.
        - `acceleration`: Must be between 0 and 256 m/s^2.
        - `jerk`: Must be between 0 and 1 m/s^3.
        - `steering_angle`: Must be between -45 and 45 degrees.
        - `steering_angle_velocity`: Must be between 0 and 1 radians/s.

        The following values are rounded to the nearest integer:
        - `speed`
        - `acceleration`
        - `jerk` (multiplied by 1000 for precision)
        - `steering_angle_velocity` (multiplied by 1000 for precision)

        The `steering_angle` value is converted to a 2-byte representation for transmission over the CAN bus.
        """

        # checks before sending Ackermann
        if 0 > ack_msg.drive.speed > 120/3.6:  # m/s   
            self.get_logger().warn('ackermann drive SPEED out of bounds')
            return None

        elif 0 > ack_msg.drive.acceleration > 255:  # m/s^2
            self.get_logger().warn('ackermann drive ACCLERATION out of bounds')
            return None

        elif 0 > ack_msg.drive.jerk > 1:  # m/s^3 
            # unsure of upper limit
            # not too fussed about assign 1 byte 
            self.get_logger().warn('ackermann drive JERK out of bounds')
            return None
        
        elif -45*pi/180 > ack_msg.drive.steering_angle > 45*pi/180:  # radians
            self.get_logger().warn('ackermann drive STEERING_ANGLE out of bounds')
            return None

        elif 0 > ack_msg.drive.steering_angle_velocity > 1:  # radians/s
            # unsure of upper limit definitely dont need more than 1
            self.get_logger().warn('ackermann drive STEERING_ANGLE_VELOCITY out of bounds')
            return None

        # format values of Ackermann
        speed = round(ack_msg.drive.speed)
        acceleration = round(ack_msg.drive.acceleration)
        jerk = round(ack_msg.drive.jerk*1000)
        steering_angle = np.float16(ack_msg.drive.steering_angle).tobytes()
        steering_angle_vel = round(ack_msg.drive.steering_angle_velocity*1000)

        # separator for steering_angle
        s_a_size = len(steering_angle)

        # Compose CAN data packet
        ackermann_vals = [
            speed,
            acceleration,
            jerk,
            steering_angle[:s_a_size//2],
            steering_angle[s_a_size//2:],
            steering_angle_vel,
            ]

        # #compress Ackermann values
        # compressed_ack_vals = compress_floats(ackermann_vals)
        
        # #separate compressed ackermann values to list of bytes for CAN
        # data_packets = [compressed_ack_vals[i:i+1] for i in range(0, len(compressed_ack_vals), 1)]

        return ackermann_vals


    def ack_to_can_publish_callback(self, ack_msg: AckermannDriveStamped):
        can_msg = CANStamped()

        # configure header
        can_header = Header()
        can_header.stamp = self.get_clock().now()
        can_header.frame_id = 'ackermann_to_can'

        # set CAN header/data/id 
        can_msg.header = can_header
        can_msg.can.id = 56 #TODO need to change to dynamic
        can_msg.can.data = self.ackermann_to_can_parser(ack_msg)

        if can_msg.can.data is not None:
            # publish CAN to topic
            self.can_pub.publish(can_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ack_to_can()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
