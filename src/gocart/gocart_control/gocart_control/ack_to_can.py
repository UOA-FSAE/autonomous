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
from fsae_interfaces.msg import CANStamped

def can_data_is_valid(data):
    from collections.abc import Sequence
    from collections.abc import Set
    from collections import UserList
    from collections import UserString
    return (
        (isinstance(data, Sequence) or
        isinstance(data, Set) or
        isinstance(data, UserList)) and
        not isinstance(data, str) and
        not isinstance(data, UserString) and
        all(isinstance(v, int) for v in data) and
        all(val >= 0 and val < 256 for val in data)
    )

def get_can_data_zero():
    ackermann_vals = np.array([
    int(0), # velocity
    int(0), # acceleration
    int(0), # jerk
    int(0), # steering_angle
    int(0), # reserved
    int(0), # angular velocity
    int(0), # reserved
    int(0)  # reserved 
    ],  
    dtype=np.uint8
    )
    return ackermann_vals.tolist()
 
#  todo create a CAN message class wrapper


class AckToCan(Node):
    def __init__(self, *vargs, **kwargs):
        super().__init__('ackermann_to_can', *vargs, **kwargs) # node name (NB: MoTec listens to this)

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
            self.AckToCan_publish_callback,    #callback function
            10                         # qos profile
        )

        # create publisher for CAN
        self.can_pub = self.create_publisher( 
            CANStamped,
            'pub_raw_can',
            10  
        )
        
        self.create_timer(1.0, self.feedback) # used to wake the thread occassionally to handle sigint signals 
        self.on_start()
        
    def feedback(self):
        pass
        # print("Hello world")
        
    def on_start(self):
        # zero all control signals
        can_msg = CANStamped()

        # configure header
        can_msg.header.frame_id = 'ackermann_to_can'

        # set CAN id/data
        can_msg.can.id = self.can_id
        print(can_data_is_valid(get_can_data_zero()))
        can_msg.can.data = get_can_data_zero()
        
        # log
        self.get_logger().info(f"INITIALIZING CONTROL SIGNALS ")
        
        # publish CAN to topic
        self.can_pub.publish(can_msg)
    
    def on_shutdown(self):
        # zero all control signals
        can_msg = CANStamped()

        # configure header
        can_msg.header.frame_id = 'ackermann_to_can'

        # set CAN id/data
        can_msg.can.id = self.can_id
        can_msg.can.data = get_can_data_zero()
        
        # log
        self.get_logger().info(f"SHUTTING DOWN - ZEROING CONTROL SIGNALS")
        
        # publish CAN to topic
        self.can_pub.publish(can_msg)
        
        
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
        if 0 > ack_msg.drive.speed or ack_msg.drive.speed > 255:  # m/s   
            self.get_logger().warn('ackermann drive SPEED out of bounds: ' + str(ack_msg.drive.speed))
            return None

        elif 0 > ack_msg.drive.acceleration or ack_msg.drive.acceleration > 255:  # m/s^2
            self.get_logger().warn('ackermann drive ACCLERATION out of bounds: ' + str(ack_msg.drive.acceleration))
            return None

        elif 0 > ack_msg.drive.jerk or ack_msg.drive.jerk > 1:  # m/s^3 
            # unsure of upper limit
            # not too fussed about assign 1 byte 
            self.get_logger().warn('ackermann drive JERK out of bounds: ' + str(ack_msg.drive.jerk))
            return None
        
        elif  ack_msg.drive.steering_angle < -30 or 30 < ack_msg.drive.steering_angle:  # degrees
            self.get_logger().warn('ackermann drive STEERING_ANGLE out of bounds: ' + str(ack_msg.drive.steering_angle))
            return None

        elif 0 > ack_msg.drive.steering_angle_velocity or ack_msg.drive.steering_angle_velocity > 1:  # radians/s
            # unsure of upper limit definitely dont need more than 1
            self.get_logger().warn('ackermann drive STEERING_ANGLE_VELOCITY out of bounds: ' + str(ack_msg.drive.steering_angle_velocity))
            return None

        # format values of Ackermann
        speed = ack_msg.drive.speed
        acceleration = ack_msg.drive.acceleration
        jerk = ack_msg.drive.jerk*100
        steering_angle = ack_msg.drive.steering_angle *4
        steering_angle_vel = ack_msg.drive.steering_angle_velocity*100
        
        # convert fro 2's compliment to signed magnitude
        if (steering_angle < 0):
            steering_angle = int(-steering_angle) | 0x80
            

    
        # Float format for steering
        # separator for steering_angle
        # steering_angle = np.float16(ack_msg.drive.steering_angle).tobytes()
        # s_a_size = len(steering_angle)
        # steering_angle_lower = int.from_bytes(steering_angle[:s_a_size//2], 'big'),
        # steering_angle_upper = int.from_bytes(steering_angle[s_a_size//2:], 'big'),
        
        ackermann_vals = np.array([
            int(speed),
            int(acceleration),
            int(jerk),
            int(steering_angle),
            int(0.0), # reserved
            int(steering_angle_vel),
            int(0),   # reserved
            int(0)    # reserved 
            ],  
            dtype=np.uint8
            )

        return ackermann_vals.tolist()


    def AckToCan_publish_callback(self, ack_msg: AckermannDriveStamped):
        can_msg = CANStamped()

        # configure header
        can_msg.header.frame_id = 'ackermann_to_can'

        # set CAN header/data/id 
        can_msg.can.id = self.can_id
        data = self.ackermann_to_can_parser(ack_msg)

        if data is not None:
            self.get_logger().info(f"DATA IS = {data}")
            can_msg.can.data = data
            # publish CAN to topic
            self.can_pub.publish(can_msg)


def shutdown_cb():
    print("[from callback] - shutting down ")


from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    
    # set up a cucstom context to handle init-shutdown cycle
    context = Context()
    rclpy.init(args=args, context=context)
    
    ack_to_can_node = AckToCan(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(ack_to_can_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nkeyboard interrupt signal intercepted")
        ack_to_can_node.on_shutdown()
    finally:
        print("\nshutting down")
        ack_to_can_node.destroy_node()
    context.shutdown()


if __name__ == '__main__':
    main()
