import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoTriangleNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)

        self.steering_pub = self.create_publisher(
            Float32,
            '/steering',
            10)

    def image_callback(self, msg):
        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Load the ArUco dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # Initialize the detector parameters
        parameters = aruco.DetectorParameters()

        detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Detect the markers
        corners, ids, rejectedImgPoints = detector.detectMarkers(cv_image)

        if ids is not None:
            odd_markers = []
            even_markers = []

            for i, corner in zip(ids, corners):
                if i % 2 == 0:
                    even_markers.append(np.mean(corner, axis=1))
                else:
                    odd_markers.append(np.mean(corner, axis=1))

            # find closest odd-even pair
            min_distance = np.inf
            closest_pair = None

            for odd in odd_markers:
                for even in even_markers:
                    dist = np.linalg.norm(odd-even)

                    if dist < min_distance:
                        min_distance = dist
                        closest_pair = (odd, even)

            if closest_pair is not None:
                midpoint = np.mean(closest_pair, axis=0)[0]
                self.get_logger().info(f'Closest midpoint: {midpoint}')

                # calculate steering percentage
                img_width = cv_image.shape[1]
                steering_percentage = ((midpoint[0] / img_width) - 0.5) * 200 # normalize to [-100, 100]
                # self.steering_pub.publish(Float32(data=steering_percentage))

                # Draw the midpoint on the image
                cv2.circle(cv_image, (int(midpoint[0]), int(midpoint[1])), 5, (0, 255, 0), -1)
                cv2.imshow("Image with Midpoint", cv_image)
                cv2.waitKey(1)

        else:
            self.get_logger().info('No ArUco markers detected')


def main(args=None):
    rclpy.init(args=args)

    node = ArucoTriangleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
