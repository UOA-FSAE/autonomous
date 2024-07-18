import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np


def get_middle_point(x1, x2):
    return np.array([
        (x1[0] + x2[0]) / 2,
        (x1[1] + x2[1]) / 2,
    ])


def find_middle_point_of_trig(point1, point2, point3):
    x_middle = (point1[0] + point2[0] + point3[0]) / 3
    y_middle = (point1[1] + point2[1] + point3[1]) / 3
    middle_point = [x_middle, y_middle]
    return middle_point


def create_boundary(points):
    sorted_points = sorted(points, key=lambda p: p[1])

    min_x_point = sorted_points.pop(-1)
    sorted_points.sort(key=lambda p: np.linalg.norm(np.array(min_x_point) - np.array(p)))

    sorted_points.insert(0, min_x_point)
    return sorted_points


def get_mid_trig_sets(set1, set2, debug_image=None):
    if (len(set1) < 2 or len(set2

                             ) < 2) and len(set1) + len(set2) < 3:
        return

    # merge lists
    set_of_points = []
    for point1, point2 in zip(set1, set2):
        set_of_points.append(point1)
        set_of_points.append(point2)

    middle_points = []
    for idx in range(0, len(set_of_points) - 2, 2):
        middle_points.append(
            find_middle_point_of_trig(set_of_points[idx],
                                      set_of_points[idx + 1],
                                      set_of_points[idx + 2]))
        middle_points.append(
            find_middle_point_of_trig(set_of_points[idx],
                                      set_of_points[idx + 1],
                                      set_of_points[idx + 3]))
        middle_points.append(
            find_middle_point_of_trig(set_of_points[idx + 1],
                                      set_of_points[idx + 2],
                                      set_of_points[idx + 3]))
        middle_points.append(
            find_middle_point_of_trig(set_of_points[idx],
                                      set_of_points[idx + 2],
                                      set_of_points[idx + 3]))

        if debug_image is not None:
            draw_trig(debug_image, set_of_points[idx],
                      set_of_points[idx + 1],
                      set_of_points[idx + 2])
            draw_trig(debug_image, set_of_points[idx],
                      set_of_points[idx + 1],
                      set_of_points[idx + 3])
            draw_trig(debug_image, set_of_points[idx + 1],
                      set_of_points[idx + 2],
                      set_of_points[idx + 3])
            draw_trig(debug_image, set_of_points[idx],
                      set_of_points[idx + 2],
                      set_of_points[idx + 3])

    return middle_points


def draw_trig(image, point1, point2, point3):
    cv2.line(image,
             (int(point1[0]), int(point1[1])),
             (int(point2[0]), int(point2[1])),
             (0, 122, 122), 2)
    cv2.line(image,
             (int(point2[0]), int(point2[1])),
             (int(point3[0]), int(point3[1])),
             (0, 122, 122), 2)
    cv2.line(image,
             (int(point3[0]), int(point3[1])),
             (int(point1[0]), int(point1[1])),
             (0, 122, 122), 2)


class ArucoMiddleNode(Node):
    def __init__(self):
        super().__init__('aruco_middle_node')
        self.bridge = CvBridge()
        self.debug = True
        self.Kp_controller = PController(2)
        self.current_point = None
        self.image_shape = None

        self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)

        self.steering_pub = self.create_publisher(
            Float32,
            '/set_steering',
            10)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.steering_percentage = 0.0
        self.current_target = [0, 0]

    def image_callback(self, msg):

        # Convert ROS image message to OpenCV image.
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.current_point = (
            int(1 * (cv_image.shape[1] / 2)),
            int(5 * (cv_image.shape[0] / 6)),
        )

        self.image_shape = cv_image.shape

        # Detect ArUco markers in image.
        corners, ids, _ = self.detector.detectMarkers(cv_image)
        even_markers = []
        odd_markers = []

        # has to be a better way of doing this
        if not 0 == len(corners):
            even_markers = [
                np.mean(marker, axis=1)[0]
                for (marker_id, marker) in zip(ids, corners) if  marker_id == 2
            ]

        if not 0 == len(corners):
            odd_markers = [
                np.mean(marker, axis=1)[0]
                for (marker_id, marker) in zip(ids, corners) if marker_id == 1
            ]

        if 0 == len(even_markers) and 0 != len(odd_markers):
            odd_boundary = create_boundary(odd_markers)
            error = self.get_target_error(target_point=(int(odd_boundary[0][0]), int(odd_boundary[0][1])),
                                          debug_img=cv_image)
            if error:
                error = error[0]
                change_steering = -100.0 - self.Kp_controller.process(error, self.current_point[0])
                steering_msg = Float32()
                steering_msg.data = change_steering
                self.steering_pub.publish(steering_msg)

            prev_point = np.array([int(odd_boundary[0][0]), cv_image.shape[0]])
            for odd_boundary_point in odd_boundary:
                cv2.line(cv_image, prev_point,
                         (int(odd_boundary_point[0]), int(odd_boundary_point[1])),
                         (0, 122, 255), 2)
                prev_point = (int(odd_boundary_point[0]), int(odd_boundary_point[1]))

            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.imshow("Aruco Middle ", cv_image)
            cv2.waitKey(1)

            return

        elif 0 != len(even_markers) and 0 == len(odd_markers):
            even_boundary = create_boundary(even_markers)
            error = self.get_target_error(target_point=(int(even_boundary[0][0]), int(even_boundary[0][1])),
                                          debug_img=cv_image)

            print(f'{error=}')
            if error:
                error = error[0]
                change_steering = -100.0 - self.Kp_controller.process(error, self.current_point[0])
                print(change_steering)
                steering_msg = Float32()
                steering_msg.data = change_steering
                self.steering_pub.publish(steering_msg)

            prev_point = np.array([int(even_boundary[0][0]), cv_image.shape[0]])
            for even_boundary_point in even_boundary:
                cv2.line(cv_image, prev_point,
                         (int(even_boundary_point[0]), int(even_boundary_point[1])),
                         (0, 122, 255), 2)
                prev_point = (int(even_boundary_point[0]), int(even_boundary_point[1]))

            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.imshow("Aruco Middle ", cv_image)
            cv2.waitKey(1)

            return

        elif 0 == len(even_markers) and 0 == len(odd_markers):
            return

        even_boundary = create_boundary(even_markers)
        odd_boundary = create_boundary(odd_markers)

        # middle points of every even and odd ID marker
        # middle_points = [
        #     get_middle_point(even_marker, odd_marker) \
        #     for even_marker in even_markers \
        #     for odd_marker in odd_markers
        # ]

        # Middle points of the bounday lines
        middle_points = [
            get_middle_point(point1, point2) \
            for (point1, point2) in zip(even_boundary, odd_boundary)
        ]

        middle_trig_points = get_mid_trig_sets(even_boundary, odd_boundary, cv_image)

        # calculate the target point and then get the error to correct for
        error = self.get_target_error(target_point=(int(middle_points[0][0]), int(middle_points[0][1])),
                                      debug_img=cv_image)

        if error:
            error = error[0]
            change_steering = self.Kp_controller.process(error, self.current_point[0])
            steering_msg = Float32()
            steering_msg.data = change_steering
            self.steering_pub.publish(steering_msg)

        if not self.debug:
            return

        print("------------------------------")
        print(cv_image.shape)
        for middle_point in middle_points:
            print(middle_point)

        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        for middle_point in middle_points:
            point = (int(middle_point[0]), int(middle_point[1]))
            cv2.circle(cv_image, point, radius=5, color=(0, 0, 255), thickness=-1)

        # odd boundary line
        prev_point = np.array([int(odd_boundary[0][0]), cv_image.shape[0]])
        for odd_boundary_point in odd_boundary:
            cv2.line(cv_image, prev_point,
                     (int(odd_boundary_point[0]), int(odd_boundary_point[1])),
                     (0, 122, 255), 2)
            prev_point = (int(odd_boundary_point[0]), int(odd_boundary_point[1]))
        #
        # # even boundary line
        prev_point = np.array([int(even_boundary[0][0]), cv_image.shape[0]])
        for even_boundary_point in even_boundary:
            cv2.line(cv_image, prev_point,
                     (int(even_boundary_point[0]), int(even_boundary_point[1])),
                     (0, 122, 255), 2)
            prev_point = (int(even_boundary_point[0]), int(even_boundary_point[1]))

        # Middle track line
        prev_point = np.array([cv_image.shape[1] // 2, cv_image.shape[0]])
        for middle_point in middle_points:
            cv2.line(cv_image, prev_point,
                     (int(middle_point[0]), int(middle_point[1])),
                     (0, 255, 0), 2)
            prev_point = (int(middle_point[0]), int(middle_point[1]))

        # Trig track line
        prev_point = np.array([cv_image.shape[1] // 2, cv_image.shape[0]])
        if middle_trig_points is not None:
            for middle_trig_point in middle_trig_points:
                cv2.line(cv_image, prev_point,
                         (int(middle_trig_point[0]), int(middle_trig_point[1])),
                         (0, 122, 122), 2)
                prev_point = (int(middle_trig_point[0]), int(middle_trig_point[1]))

        cv2.circle(cv_image, self.current_point, radius=5, color=(0, 255, 0), thickness=-1)

        # cv2.circle(cv_image, (int(cv_image.shape[1]/2), int(cv_image.shape[0]/2)),
        #            radius=15, color=(0, 0, 255), thickness=-1)

        cv2.imshow("Aruco Middle ", cv_image)
        cv2.waitKey(1)

    def get_target_error(self, target_point, debug_img=None):
        # draws a line across the screen and gets the intersect at that point
        def line(p1, p2):
            A = (p1[1] - p2[1])
            B = (p2[0] - p1[0])
            C = (p1[0] * p2[1] - p2[0] * p1[1])
            return A, B, -C

        def intersection(L1, L2):
            D = L1[0] * L2[1] - L1[1] * L2[0]
            Dx = L1[2] * L2[1] - L1[1] * L2[2]
            Dy = L1[0] * L2[2] - L1[2] * L2[0]
            if D != 0:
                x = Dx / D
                y = Dy / D
                return x, y
            else:
                return False

        L1 = line(target_point,
                  (self.image_shape[1] // 2,
                   self.image_shape[0]))
        L2 = line((0, self.current_point[1]), (self.image_shape[1], self.current_point[1]))
        R = intersection(L1, L2)

        if debug_img is not None and R:
            cv2.line(debug_img,
                     (0, self.current_point[1]),
                     (self.image_shape[1], self.current_point[1]),
                     (255, 0, 0), 1)
            cv2.line(
                debug_img,
                (int(R[0]), int(R[1])),
                self.current_point,
                (0, 255, 0), 2
            )

            print(f'{R=}')

        return R


class PController:
    def __init__(self, Kp):
        self.Kp = Kp

    def process(self, target, current):
        control_effort = target - current
        return control_effort * self.Kp


def main(args=None):
    rclpy.init(args=args)

    node = ArucoMiddleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
