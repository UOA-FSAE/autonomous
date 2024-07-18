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


def get_angle_from_number(num):
    return (num + 1) * 45


def draw_line(image, angle_degrees):
    angle_radians = np.deg2rad(angle_degrees)
    center_x = image.shape[1] // 2
    end_x = int(center_x + image.shape[1] / 2 * np.tan(angle_radians))
    end_y = image.shape[0]
    print(center_x, image.shape[0], end_x, end_y)
    cv2.line(image, (center_x, image.shape[0]), (end_x, end_y), color=(0, 0, 255), thickness=10)


def debug_show(image, markers):
    if markers[0] is not None and markers[1] is not None:
        cv2.aruco.drawDetectedMarkers(image, markers[0], markers[1])

        closest_pair = detect_center_of_markers(markers[0], markers[1])
        # Add a cirlce to the image in the center of the two images
        if closest_pair is not None:
            center_x = int((closest_pair[0][0][0] + closest_pair[1][0][0]) / 2)
            center_y = int((closest_pair[0][0][1] + closest_pair[1][0][1]) / 2)
            cv2.circle(image, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

            middle_x = image.shape[1] // 2
            cv2.line(image, (middle_x, image.shape[0]), (center_x, center_y), color=(0, 0, 255), thickness=10)

    cv2.imshow('Aruco Markers', image)
    cv2.waitKey(1)


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
            '/set_steering',
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

        debug_show(cv_image, (corners, ids))

        if corners is None or ids is None:
            return
        # Detect center of closest pair of markers.
        closest_pair = detect_center_of_markers(corners, ids)

        if closest_pair is not None:
            # Calculate steering command based on position of center of closest pair of markers.
            self.current_target = np.mean(closest_pair, axis=0)[0]
            self.steering_percentage = ((self.current_target[0] / cv_image.shape[1]) - 0.5) * -2
            print(self.steering_percentage)

            # Publish steering command.
            msg = Float32()
            msg.data = float(self.steering_percentage)
            self.steering_pub.publish(msg)
        # else:
        # Log message if no ArUco markers detected.
        # self.get_logger().info('No ArUco markers set detected')


def main(args=None):
    rclpy.init(args=args)

    node = ArucoTriangleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
