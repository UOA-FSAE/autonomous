import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from moa_msgs.nsg import Cone, ConeMap
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detection')
        self.bridge = CvBridge()
        self.debug = True

        # subscriptions
        self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)

        self.cone_pub = self.create_publisher(
            ConeMap,
            'cone_detect',
            10)

    def image_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
