#!/usr/bin/env python3
# Python imports
from typing import Optional
import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from rcl_interfaces.msg import ParameterDescriptor

# Ros Imports
from ackermann_msgs.msg import AckermannDriveStamped
from moa_msgs.msg import CANStamped


class ack_to_can(Node):
    def __init__(self):
        super().__init__('ackermann_to_can') # node name (NB: MoTec listens to this)

        # init ros_arg parameters
        self.declare_parameter('can_id', 
                               300, 
                               ParameterDescriptor(description= 'The frame ID for the CAN messages sent to the car'))
        
        self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        self.get_logger().info(f'the value of can_id is {self.can_id}')

        # create subscriber for ackermann input
        self.subscription = self.create_subscription(
            AckermannDriveStamped,     # msg type
            'cmd_vel',                 # topic receiving from
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
        if 0 > ack_msg.drive.speed or ack_msg.drive.speed > 120/3.6:  # m/s   
            self.get_logger().warn('ackermann drive SPEED out of bounds')
            return None

        elif 0 > ack_msg.drive.acceleration or ack_msg.drive.acceleration > 255:  # m/s^2
            self.get_logger().warn('ackermann drive ACCLERATION out of bounds')
            return None

        elif 0 > ack_msg.drive.jerk or ack_msg.drive.jerk > 1:  # m/s^3 
            # unsure of upper limit
            # not too fussed about assign 1 byte 
            self.get_logger().warn('ackermann drive JERK out of bounds')
            return None
        
        elif -45*pi/180 > ack_msg.drive.steering_angle or ack_msg.drive.steering_angle > 45*pi/180:  # radians
            self.get_logger().warn('ackermann drive STEERING_ANGLE out of bounds')
            return None

        elif 0 > ack_msg.drive.steering_angle_velocity or ack_msg.drive.steering_angle_velocity > 1:  # radians/s
            # unsure of upper limit definitely dont need more than 1
            self.get_logger().warn('ackermann drive STEERING_ANGLE_VELOCITY out of bounds')
            return None

        # format values of Ackermann
        speed = ack_msg.drive.speed
        acceleration = ack_msg.drive.acceleration
        jerk = ack_msg.drive.jerk*100
        steering_angle = np.float16(ack_msg.drive.steering_angle).tobytes()
        steering_angle_vel = ack_msg.drive.steering_angle_velocity*100

        # separator for steering_angle
        s_a_size = len(steering_angle)

        # Compose CAN data packet
        ackermann_vals = np.array([
            speed,
            acceleration,
            jerk,
            int.from_bytes(steering_angle[:s_a_size//2], 'big'),
            int.from_bytes(steering_angle[s_a_size//2:], 'big'),
            steering_angle_vel,
            0,
            0], 
            dtype=np.uint8
            )

        return ackermann_vals


    def ack_to_can_publish_callback(self, ack_msg: AckermannDriveStamped):
        can_msg = CANStamped()

        # configure header
        can_msg.header.frame_id = 'ackermann_to_can'

        # set CAN header/data/id 
        can_msg.can.id = self.can_id
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
