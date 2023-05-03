import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class scrut_mission_node(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, 
            'cmd_val', 
            10
        )

        self.timer = self.create_timer(timer_period = 5, callback = self.timer_callback)

    def timer_callback(self):
        pass

    def form_msg(self, steering_angle:float, steering_angle_vel:float, speed:float, acceleration:float, jerk:float) -> AckermannDriveStamped:
        ack_msg = AckermannDriveStamped()
        
        ack_msg.drive.steering_angle = steering_angle
        ack_msg.drive.steering_angle_vel = steering_angle_vel
        ack_msg.drive.speed = speed
        ack_msg.drive.acceleration = acceleration
        ack_msg.drive.jerk = jerk 
        
        return ack_msg


def main(args=None):
    rclpy.init(args=args)

    scrut_node = scrut_mission_node()

    rclpy.spin(scrut_node)

    scrut_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()