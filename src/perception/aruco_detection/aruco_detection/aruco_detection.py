import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose

import cv2
from cv_bridge import CvBridge
#If error happens, pip install opencv-contrib-python
import cv2.aruco as aruco
import numpy as np

from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone


class ArucoDetectionNode(Node):

    def __init__(self):
        super().__init__('Aruco_detection_node')
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo,'zed/zed_node/rgb/camera_info',self.camera_callback,10)
        self.create_subscription(PoseStamped,'zed/zed_node/pose',self.pose_callback,10)
        self.create_subscription(Image,'/zed/zed_node/rgb/image_rect_color',self.image_callback,10)
        self.publisher = self.create_publisher(ConeMap, 'cone_detection', 10)
        self.localization_publisher = self.create_publisher(Pose, 'car_position', 5)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        #THe length of marker in real life, Unit:m
        marker_size = 0.19
        self.marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],[marker_size / 2, marker_size / 2, 0],[marker_size / 2, -marker_size / 2, 0],[-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        self.edit_msg = False
        self.aruco_msg = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.get_logger().info("class initialized")

    def image_callback(self, msg):
        self.get_logger().info("publishing cone map soon")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        #Visualization of detection result
        #detected_markers = aruco.drawDetectedMarkers(cv_image,corners,ids)
        #if np.all(ids is not None):
        #    cv2.imshow('out', detected_markers)

        if (self.camera_matrix is not None) and (self.dist_coeffs is not None):
            #Doesn't allow further processing unless the camera info is known
            if(self.edit_msg):
                if np.all(ids is not None):
                    for indx,id in enumerate(ids):
                        single_cone = Cone()
                        idc = id[0]
                        _,rvec,tvec = cv2.solvePnP(self.marker_points,corners[indx],self.camera_matrix,self.dist_coeffs)

                        single_cone.id = indx
                        single_cone.confidence = 100.0
                        # assuming id 1 is blue cone and id 2 is yellow cone
                        single_cone.colour = 0 if idc==1 else 2 if idc==2 else 3

                        # x-axis is forward/backward
                        single_cone.pose.pose.position.x = float(tvec[0][0])
                        # y-axis is left/right
                        single_cone.pose.pose.position.y = float(tvec[2][0])
                        # z-axis is up/down
                        single_cone.pose.pose.position.z = float(tvec[1][0])
                        single_cone.radius = 1.0
                        single_cone.height = 1.0
                        # Only sending blue cone and yellow cone
                        if single_cone.colour == 0 or single_cone.colour == 2:
                            self.aruco_msg.cones.append(single_cone)
                    self.get_logger().info("POTENTIAL MARKERS DETECTED!!")
                
                self.publisher.publish(self.aruco_msg)
                self.edit_msg = False

    def pose_callback(self,msg):
        self.aruco_msg = ConeMap()
        localization_cone = Cone()
        localization_pose = Pose()
        localization_cone.id = 99999;
        euler = self.convert_quaternion_to_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        localization_pose.position.x = -msg.pose.position.y
        localization_pose.position.y = msg.pose.position.x
        localization_pose.orientation.w = euler[1]
        localization_cone.pose.pose = localization_pose
        self.aruco_msg.cones.append(localization_cone)
        self.edit_msg = True
        self.localization_publisher.publish(localization_pose)

    def camera_callback(self,msg):
        self.camera_matrix = np.reshape(msg.k, (3,3))
        self.dist_coeffs = np.array(msg.d)

    def convert_quaternion_to_euler(self, x, y, z, w):
        pitch = np.arcsin(2*(w*y-z*x))
        yaw = np.arctan2(2*(w*z+x*y), 1-(2*(y**2 + z**2)))
        roll = np.arctan2(2*(w*x+y*z), 1-(2*(x**2+y**2)))
        return pitch, yaw, roll


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetectionNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
