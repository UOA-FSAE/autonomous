# This node aims to explicitly extract the localization data from the cone detection node to pop the localization data as fast as possible for controller

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from moa_msgs.msg import Track


class Localization(Node):
# TODO: Test in real life to see how to make localization data more accurate: type of approch I can think of: Bayessian filtering, Sensor fusion, Get data from Cone Map, Or simply better sensor setup on go-kart
    def __init__(self):
        super().__init__('localization')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.left_cones = []
        self.right_cones = []
        self.localization_publisher = self.create_publisher(Pose, 'car_position', 5)
        self.cone_detection_subscription_left_track = self.create_subscription(
            Track,
            'left_track',
            self.left_cone_map_callback,
            qos_profile)
        
        self.subscription_right_cone_map = self.create_subscription(
            Track, 
            'right_track',  
            self.right_cone_map_callback, 
            qos_profile)
        
        self.get_logger().info('Localization node started')

    # def listener_callback(self, msg: Track):
    #     car_cone = msg.cones[0]
    #     car_pose = car_cone.pose.pose
    #     self.localization_publisher.publish(car_pose)
    #     self.get_logger().info("Car localization data published")

    def left_cone_map_callback(self, msg:Track) -> None:
        self.left_cones = msg.cones

    def right_cone_map_callback(self, msg:Track) -> None:
        self.right_cones = msg.cones


def main(args=None):
    rclpy.init(args=args)

    localizer = Localization()

    rclpy.spin(localizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
