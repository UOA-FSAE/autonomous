#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from moa_msgs.msg import ConeMap
from geometry_msgs.msg import Pose

class car_position(Node):
    def __init__(self):
        super().__init__("car_position")

        self.create_subscription(ConeMap, "cone_map", self.callback, 10)
        self.car_position_pub = self.create_publisher(Pose, "car_position", 10)

    def callback(self, msg:ConeMap):   
        msg = msg.cones[0].pose.pose
        self.car_position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = car_position()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
