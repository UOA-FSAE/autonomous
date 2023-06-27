import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


def detect_center_of_markers(corners, ids):
    """
    Function to detect the centers of ArUco markers in an image.

    Args:
        corners (list): List of detected marker corners.
        ids (list): List of detected marker ids.

    Returns:
        tuple: Tuple of the centers of the closest pair of odd and even markers.
    """
    # Lists to hold markers with odd and even ids.
    odd_markers = []
    even_markers = []

    # Loop over detected marker corners and ids.
    for i, corner in zip(ids, corners):
        # Check if marker id is odd or even and append to appropriate list.
        if i % 2 == 0:
            even_markers.append(np.mean(corner, axis=1))
        else:
            odd_markers.append(np.mean(corner, axis=1))

    # Variables to hold closest pair of odd and even markers and their distance.
    min_distance = np.inf
    closest_pair = None

    # Loop over pairs of odd and even markers.
    for odd in odd_markers:
        for even in even_markers:
            # Calculate Euclidean distance between odd and even marker.
            dist = np.linalg.norm(odd - even)

            # If distance is less than current minimum, update minimum distance and closest pair.
            if dist < min_distance:
                min_distance = dist
                closest_pair = (odd, even)

    # Return closest pair of markers.
    return closest_pair


class ArucoTriangleNode(Node):
    def __init__(self):
        """
        Constructor for the ArucoTriangleNode class.

        Initializes the node, CvBridge, subscriptions, publishers and ArUco detector.
        """
        super().__init__('aruco_triangle_node')
        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)

        self.steering_pub = self.create_publisher(
            Float32,
            '/moa/target_steering',
            10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.steering_percentage = 0.0
        self.current_target = [0, 0]

    def image_callback(self, msg):
        """
        Callback for image messages from the subscribed topic.

        Args:
            msg (sensor_msgs.msg.Image): The incoming image message.
        """
        # Convert ROS image message to OpenCV image.
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers in image.
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        # Detect center of closest pair of markers.
        closest_pair = detect_center_of_markers(corners, ids)

        if closest_pair is not None:
            # Calculate steering command based on position of center of closest pair of markers.
            self.current_target = np.mean(closest_pair, axis=0)[0]
            self.steering_percentage = ((self.current_target[0] / cv_image.shape[
                1]) - 0.5) * 200  # normalize to [-100, 100]

            # Publish steering command.
            self.steering_pub.publish(f"Float32(data={self.steering_percentage})")
        else:
            # Log message if no ArUco markers detected.
            self.get_logger().info('No ArUco markers set detected')


def main(args=None):
    rclpy.init(args=args)

    node = ArucoTriangleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
