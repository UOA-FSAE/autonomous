import rclpy
from rclpy.node import Node
from time import sleep
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class testNode(Node):
    def __init__(self):
        super().__init__('testNode')
        self.sub = self.create_subscription(AckermannDriveStamped, 'cmd_val', self.msg_callback,10)
        self.pub = self.create_publisher(AckermannDriveStamped, 'moa/curr_vel',10)
        self.received = []
        self.real = False
    def msg_callback(self, msg):
        self.received.append(msg)
        sleep(1)
        if self.real:
            for i in range(1,5):
                rand = random.random()
                noise_msg = AckermannDrive(speed = rand, accel = rand)
                self.pub.publish(AckermannDriveStamped(drive = noise_msg))
                sleep(0.5)

        self.pub.publish(AckermannDriveStamped(drive = msg.drive))
        self.get_logger().info(f'received msg {msg}')


def main(args=None):
    rclpy.init(args=args)

    test_node = testNode()

    rclpy.spin(test_node)

    test_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()