import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageThrottlerNode(Node):
    def __init__(self):
        super().__init__('image_throttled_node')

        # parameter n is one image outputted per n input images(default 30)
        self.declare_parameter('n', 60)
        self.n = self.get_parameter('n').get_parameter_value().integer_value

        self.subscriber = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(Image, '/image_throttled', 10)

        self.counter = 0
        self.get_logger().info(f'ImageThrottlerNode started, passing every {self.n}th image.')

    def image_callback(self, msg):
        self.counter += 1
        if self.counter >= self.n:
            self.publisher.publish(msg)
            self.get_logger().debug(f'Publishing image #{self.counter}')
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImageThrottlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
