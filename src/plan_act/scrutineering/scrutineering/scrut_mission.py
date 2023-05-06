import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from math import pi


def deg_rads(degs):
    return degs * 180/pi


class scrut_mission_node(Node):
    def __init__(self):
        super().__init__('scrutineering_mission')
        self.publisher = self.create_publisher(
            AckermannDriveStamped, 
            'cmd_val', 
            10
        )

        self.subscriber = self.create_subscription(
            AckermannDriveStamped,
            "moa/curr_vel",
            self.read_state,
            10
        )

        self.curr_state = None


    def read_state(self, msg):
        self.curr_state = msg.drive

    def send_payloads(self, payloads):
        timeout = Duration(seconds=30)
        start_time = self.get_clock().now()
        end_time = start_time + timeout

        for data in payloads:
            msg = AckermannDriveStamped(drive = data)
            self.publisher.publish(msg)
            while self.curr_state != msg and self.get_clock().now() < end_time:
                pass
            if self.curr_state != msg:
                self.get_logger().error("timeout: command didn't complete")
                raise TimeoutError()
            
    def steer_test(self) -> None:
        payloads = [
            AckermannDrive(steering_angle=deg_rads(-45.0),steering_angle_velocity=deg_rads(36.0)),
            AckermannDrive(steering_angle=deg_rads(0.0),steering_angle_velocity=deg_rads(36.0)),
            AckermannDrive(steering_angle=deg_rads(45.0),steering_angle_velocity=deg_rads(36.0)),
            AckermannDrive(steering_angle=deg_rads(0.0),steering_angle_velocity=deg_rads(36.0))
            ]
        
        self.get_logger().info("commencing steering test")
        self.send_payloads(payloads)        
    
    def accel_test(self) -> None:
        payloads = [
            AckermannDrive(speed=5.0,acceleration=5.0,jerk=1.0)
            ]
        
        self.get_logger().info("commencing acceleration test")
        self.send_payloads(payloads)   

    def brake_test(self) -> None:
        payloads = [
            AckermannDrive(speed=0.0,acceleration=5.0,jerk=1.0)
            ]
        
        self.get_logger().info("commencing brake test")
        self.send_payloads(payloads)

    def ebs_test(self) -> None:
        payloads = [
            AckermannDrive(speed=5.0,acceleration=5.0,jerk=1.0)
            ]
        
        self.get_logger().info("commencing EBS test")
        self.send_payloads(payloads)


def main(args=None):
    rclpy.init(args=args)

    scrut_node = scrut_mission_node()
    
    scrut_node.steer_test()
    scrut_node.accel_test()
    scrut_node.brake_test()
    scrut_node.ebs_test()

    scrut_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()