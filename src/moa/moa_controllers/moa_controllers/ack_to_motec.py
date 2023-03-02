#/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs import AckermannDriveStamped

class ack_to_mot(Node):
    def __init__(self):
        super().__init__('ackermann_to_motec') #node name

        #create subscriber
        self.subscription = self.create_subscription(
            AckermannDriveStamped,     #msg type
            'cmd_val',                 #topic receiving from
            self.listener_callback,    #callback function
            10                         #qos profile
        )

    def listener_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = ack_to_mot()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
