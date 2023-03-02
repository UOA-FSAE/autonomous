#/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ack_to_mot(Node):
    def __init__(self):
        super().__init__('ackermann_to_motec') #node name
        

def main(args=None):
    rclpy.init(args=args)

    node = ack_to_mot()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
