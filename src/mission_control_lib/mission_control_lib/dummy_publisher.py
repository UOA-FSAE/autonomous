import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        
        # Create a publisher to DUMMY_TOPIC
        self.publisher_ = self.create_publisher(String, 'DUMMY_TOPIC', 10)
        self.timer = self.create_timer(1.0, self.publish_dummy)
        self.count = 0

    def publish_dummy(self):
        msg = String()
        self.count += 1
        msg.data = 'Hello from dummy publisher!' + str(self.count)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    np_node = DummyPublisher()
    rclpy.spin(np_node)
    
    np_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()