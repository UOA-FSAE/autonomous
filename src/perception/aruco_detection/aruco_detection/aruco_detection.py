import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import cv2
#If error happens, pip install opencv-contrib-python
import cv2.aruco as aruco
import numpy as np

from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone


class ArucoDetectionNode(Node):

    def __init__(self):
        super().__init__('Aruco detection node')
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo,'/zed2i/zed_node/rgb/camera_info',self.camera_callback,10)
        self.create_subscription(PoseStamped,'/zed2i/zed_node/pose',self.pose_callback,10)
        self.create_subscription(Image,'/zed2i/zed_node/rgb/image_rect_color',self.image_callback,10)
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        #THe length of marker in real life, Unit:m
        marker_size = 0.05
        self.marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],[marker_size / 2, marker_size / 2, 0],[marker_size / 2, -marker_size / 2, 0],[-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        self.edit_msg = False
        self.aruco_msg = None
        self.camera_matrix = None
        self.dist_coeffs = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        #Visualization of detection result
        aruco.drawDetectedMarkers(cv_image,corners,ids)

        if np.all(ids is not None) and (self.camera_matrix is not None) and (self.dist_coeffs is not None):
            if(self.edit_msg):
                single_cone = Cone()
                for indx,id in enumerate(ids):
                    _,rvec,tvec = cv2.solvePnP(self.marker_points,corners[indx],self.camera_matrix,self.dist_coeffs)

                    single_cone.id = indx
                    single_cone.confidence = 100
                    single_cone.colour = id

                    single_cone.pose.pose.position.x = tvec[0]
                    single_cone.pose.pose.position.y = tvec[1]
                    single_cone.pose.pose.position.z = tvec[2]
                    single_cone.radius = 1
                    single_cone.height = 1
                    self.aruco_msg.cones.append(single_cone)

        self.publisher.publish(self.aruco_msg)
        self.edit_msg = False

    def pose_callback(self,msg):
        self.camera_matrix = msg.K
        self.dist_coeffs = msg.D

    def camera_callback(self,msg):
        self.aruco_msg = ConeMap()
        localization_cone = Cone()
        localization_cone.id = 99999;
        localization_cone.pose.pose.position.x = msg.pose.position.x
        localization_cone.pose.pose.position.y =  msg.pose.position.y
        localization_cone.pose.pose.orientation.w =  msg.pose.orientation.w
        self.aruco_msg.cones.append(localization_cone)
        self.edit_msg = True

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetectionNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
