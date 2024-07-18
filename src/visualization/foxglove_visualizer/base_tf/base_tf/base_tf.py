import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        #self.broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(TransformStamped, 'base_tf', 10)
        self.timer = self.create_timer(1.0, self.publish_transform)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global frame'
        t.child_frame_id = 'local frame'

        # No translation
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        #self.broadcaster.sendTransform(t)
        self.publisher_.publish(t);


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
