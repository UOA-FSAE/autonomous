#!/usr/bin/env python3
# Python imports
import rclpy
from rclpy.node import Node
from moa_msgs.msg import ConeMap
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header


class Acceleration_algorithm(Node):
    def __init__(self):
        super().__init__("Acceleration")
        self.get_logger().info("Acceleration Started")

        ## Current speed setting
        self.current_speed = 20/3.6 #In m/s

        # subscribe to Cone detection result
        self.create_subscription(ConeMap, "cone_detection", self.main_hearback, 5)

        self.drive_pub = self.create_publisher(AckermannDrive, "/drive", 5)
        self.drive_vis_pub = self.create_publisher(AckermannDrive, "/drive_vis", 5)
        self.cmd_vel_pub = self.create_publisher(AckermannDriveStamped, "cmd_vel", 5)

        #Start the car
        self.publish_ackermann()


    def main_hearback(self, msg: ConeMap):
        #If there are no cones, make throttle to 0
        if len(msg.cones) == 0:
            self.current_speed = 0.0
            self.publish_ackermann()



    def publish_ackermann(self):

        args1 = {"steering_angle": 0.0,
                "steering_angle_velocity": 0.0,
                "speed": float(self.current_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg1 = AckermannDrive(**args1)

        msg_cmd_vel = self.convert_to_stamped(msg1)
        self.cmd_vel_pub.publish(msg_cmd_vel)
        self.drive_pub.publish(msg1)
        self.drive_vis_pub.publish(msg1)

    def convert_to_stamped(self, ackermann_msgs):
        stamped_msg = AckermannDriveStamped()
        stamped_msg.header = Header()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()  # Set the current timestamp
        stamped_msg.header.frame_id = '0'  # Set the appropriate frame ID
        stamped_msg.drive = ackermann_msgs
        return stamped_msg

 
def main(args=None):
    rclpy.init(args=args)

    accelrator = Acceleration_algorithm()

    rclpy.spin(accelrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    accelrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
