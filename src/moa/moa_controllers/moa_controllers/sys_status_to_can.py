import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from typing import Optional

from std_msgs.msg import UInt8
from moa_msgs.msg import CANStamped

import numpy as np

class sys_status_to_can(Node):
    def __init__(self):
        super().__init__("system_status_to_can")

        # init ros_arg parameters
        self.declare_parameter('can_id', 
                               300, 
                               ParameterDescriptor(description= 'The frame ID for the CAN messages sent to the car'))
        
        self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        self.get_logger().info(f'the value of can_id is {self.can_id}')

        # subscriber for system status
        self.create_subscription(
            UInt8,
            "sys_status",
            self.callback,
            10
        )

        # publisher for can
        self.canstamped_pub = self.create_publisher(
            CANStamped,
            "pub_raw_can",
            10
        )
    
    def uint8_to_can(self, msg: UInt8) -> Optional[CANStamped]:
        # check validity of data
        if msg.data < 0 or msg.data > 4:
            self.get_logger().info(f"system status out of bounds {msg.data}")
            return None
        
        # convert data
        system_status = np.array([
            msg.data], 
            dtype=np.uint8
            )
    
        return system_status

    
    def callback(self, msg: UInt8):
        can_msg = CANStamped()

        # configure header
        can_msg.header.frame_id = 'system_status_to_can'

        # set CAN id 
        can_msg.can.id = self.can_id
        data = self.uint8_to_can(msg)

        if data is not None:
            can_msg.can.data = data
            # publish CAN to topic
            self.canstamped_pub.publish(can_msg)



def main(args=None):
    rclpy.init(args=args)

    node = sys_status_to_can()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()