import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from math import pi


def deg_rads(degs):
    return degs * 180/pi


class scrut_mission_node(Node):
    def __init__(self):
        super().__init__('scrut_mission_node')
        self.declare_parameters('', [
            ('timeout', 30),
            ('max_steering_angle', 0.0),
            ('steering_angle_velocity', 0.0),
            ('max_speed', 0.0),
            ('max_acceleration', 0.0),
            ('jerk', 0.0),
        ])
        
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
        timeout = self.get_parameter('timeout').value
        timeout = Duration(seconds=timeout)
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
        max_steer_angle = self.get_parameter('max_steering_angle').value
        steer_angle_vel = self.get_parameter('steering_angle_velocity').value

        payloads = [
            AckermannDrive(steering_angle=deg_rads(-max_steer_angle), 
                           steering_angle_velocity=deg_rads(steer_angle_vel)),
            AckermannDrive(steering_angle=deg_rads(0.0), 
                           steering_angle_velocity=deg_rads(steer_angle_vel)),
            AckermannDrive(steering_angle=deg_rads(max_steer_angle), 
                           steering_angle_velocity=deg_rads(steer_angle_vel)),
            AckermannDrive(steering_angle=deg_rads(0.0), 
                           steering_angle_velocity=deg_rads(steer_angle_vel))
            ]
        
        self.get_logger().info("commencing steering test")
        self.send_payloads(payloads)        
    
    def accel_test(self) -> None:
        max_speed = self.get_parameter('max_speed').value
        max_accel = self.get_parameter('max_acceleration').value
        max_jerk = self.get_parameter('max_jerk').value

        # remove after test
        self.get_logger().info(f"max_speed {max_speed}")
        self.get_logger().info(f"max_accel {max_accel}")
        self.get_logger().info(f"max_jerk {max_jerk}")
      

        payloads = [
            AckermannDrive(speed=max_speed,acceleration=max_accel,jerk=max_jerk)
            ]
        
        self.get_logger().info("commencing acceleration test")
        self.send_payloads(payloads)   

    def brake_test(self) -> None:
        max_accel = self.get_parameter('max_acceleration').value
        max_jerk = self.get_parameter('max_jerk').value
        
        # remove after test
        self.get_logger().info(f"max_accel {max_accel}")
        self.get_logger().info(f"max_jerk {max_jerk}")

        payloads = [
            AckermannDrive(speed=0.0,acceleration=max_accel,jerk=max_jerk)
            ]
        
        self.get_logger().info("commencing brake test")
        self.send_payloads(payloads)

    def ebs_test(self) -> None:
        max_speed = self.get_parameter('max_speed').value
        max_accel = self.get_parameter('max_acceleration').value
        max_jerk = self.get_parameter('max_jerk').value

        # remove after test
        self.get_logger().info(f"max_speed {max_speed}")
        self.get_logger().info(f"max_accel {max_accel}")
        self.get_logger().info(f"max_jerk {max_jerk}")

        payloads = [
            AckermannDrive(speed=max_speed,acceleration=max_accel,jerk=max_jerk)
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