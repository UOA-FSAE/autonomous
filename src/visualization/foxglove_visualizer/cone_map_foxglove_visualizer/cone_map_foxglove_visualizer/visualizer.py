import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3, Pose
from geometry_msgs.msg import TransformStamped
from moa_msgs.msg import Track
import numpy as np

class ConePublisher(Node):
    def __init__(self):
        self.left_cones = []
        self.right_cones = []

        super().__init__('cone_publisher')
        self.markers_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_cones', 10)
        self.localization_marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_car', 10)
        self.frame_publisher_ = self.create_publisher(TransformStamped, 'base_tf', 10)
        self.subscription_left_cone_map = self.create_subscription(Track, 'left_track',  self.left_cone_map_callback, 10)
        self.subscription_right_cone_map = self.create_subscription(Track, 'right_track',  self.right_cone_map_callback, 10)
        self.subscription_localization = self.create_subscription(Pose, 'car_position', self.localization_callback, 10)
        self.get_logger().info("Cone Map Visualization Initialization Completed")

    def left_cone_map_callback(self, msg):
        self.left_cones = msg.cones

    def right_cone_map_callback(self, msg):
        self.right_cones = msg.cones

    def localization_callback(self, msg):
        self.publish_cones()
        self.publish_transform(msg)
        self.publish_car(msg)

    def convert_rotation_to_quaternion(self, angle):
        qw = np.cos(angle / 2)
        qx = np.sin(angle / 2) * 0
        qy = np.sin(angle / 2) * 0
        qz = np.sin(angle / 2) * 1
        return qx, qy, qz, qw

    def publish_transform(self, localization_pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global_frame'
        t.child_frame_id = 'local_frame'

        # No translation
        t.transform.translation.x = localization_pose.position.x
        t.transform.translation.y = localization_pose.position.y
        t.transform.translation.z = localization_pose.position.z

        # No rotation (identity quaternion)
        t.transform.rotation.x = self.convert_rotation_to_quaternion(localization_pose.orientation.w)[0]
        t.transform.rotation.y = self.convert_rotation_to_quaternion(localization_pose.orientation.w)[1]
        t.transform.rotation.z = self.convert_rotation_to_quaternion(localization_pose.orientation.w)[2]
        t.transform.rotation.w = self.convert_rotation_to_quaternion(localization_pose.orientation.w)[3]

        self.frame_publisher_.publish(t)

    def publish_car(self, pose_of_car):
        Markers = MarkerArray()
        is_localization = True
        id_assign = 0
        Markers.markers.append(self.delete_all_cone())
        Markers.markers.append(self.convert_to_visualization(pose_of_car.position, is_localization, id_assign, colour=3))
        Markers.markers[1].pose.orientation.x = self.convert_rotation_to_quaternion(pose_of_car.orientation.w)[0]
        Markers.markers[1].pose.orientation.y = self.convert_rotation_to_quaternion(pose_of_car.orientation.w)[1]
        Markers.markers[1].pose.orientation.z = self.convert_rotation_to_quaternion(pose_of_car.orientation.w)[2]
        Markers.markers[1].pose.orientation.w = self.convert_rotation_to_quaternion(pose_of_car.orientation.w)[3]
        self.localization_marker_publisher.publish(Markers)

    def publish_cones(self):
        Markers = MarkerArray()
        is_localization = False
        id_assign = 1
        Markers.markers.append(self.delete_all_cone())
        colour = 0
        for cone in self.left_cones:
            Markers.markers.append(self.convert_to_visualization(cone, is_localization, id_assign, colour))
            id_assign += 1
        for cone in self.right_cones:
            Markers.markers.append(self.convert_to_visualization(cone, is_localization, id_assign, colour+2))
            id_assign += 1

        print("COunt: ", len(Markers.markers))
        self.markers_publisher_.publish(Markers)

    def convert_to_visualization(self, cone_point, is_localization, id_assign, colour):
        marker = Marker()
        marker.header.frame_id = "global_frame"  # Adjust the frame ID as needed
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "cone"
        marker.id = id_assign
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position = cone_point

        if is_localization:
            # Need to do later
            # marker.pose.orientation.x = self.convert_rotation_to_quaternion(cone.orientation.w)[0]
            # marker.pose.orientation.y = self.convert_rotation_to_quaternion(cone.orientation.w)[1]
            # marker.pose.orientation.z = self.convert_rotation_to_quaternion(cone.orientation.w)[2]
            # marker.pose.orientation.w = self.convert_rotation_to_quaternion(cone.orientation.w)[3]

            marker.scale = Vector3(x=0.5, y=0.2, z=0.3)  # Scale of the cone (x, y, z)

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha (opacity)

        else:
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)  # Scale of the cone (x, y, z)

            if colour == 0:
                # Yellow
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # Alpha (opacity)
            elif colour == 1:
                # Orange
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0  # Alpha (opacity)
            elif colour == 2:
                # Blue
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0  # Alpha (opacity)
            else:
                # Yellow
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # Alpha (opacity)

        marker.lifetime.sec = 0

        return marker

    def delete_all_cone(self):
        marker = Marker()
        marker.header.frame_id = "global_frame"  # Adjust the frame ID as needed
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "cone"
        marker.id = -1
        marker.type = Marker.CUBE
        marker.action = Marker.DELETEALL

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale = Vector3(x=1.0, y=1.0, z=1.0)  # Scale of the cone (x, y, z)

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (opacity)

        marker.lifetime.sec = 0

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ConePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
