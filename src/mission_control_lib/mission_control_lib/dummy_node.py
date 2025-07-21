import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mission_control_lib.subscription_manager import ManageSubscription

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

        # Create a managed subscription to DUMMY_TOPIC
        ManageSubscription(
            node=self,
            topic='DUMMY_TOPIC',
            msg_type=String,
            callback=self.dummy_callback,
            mission_array= [0, 2, 4]  # enabled for missions 0, 2, 4
        )

    def dummy_callback(self, msg):
        self.get_logger().info(f"Received under mission control: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    np_node = DummyNode()
    rclpy.spin(np_node)
    
    np_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()