import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header
from rclpy.callback_groups import ReentrantCallbackGroup


class mock_stimulus_node(Node):
    
    def __init__(self, *vargs, **kwargs):
        super().__init__("stimulus_node", *vargs, **kwargs)
        
        # Create a separate callback group for shutdown
        self.shutdown_callback_group = ReentrantCallbackGroup()
        
        # Create guard condition with callback
        self.shutdown_guard = GuardCondition(
            self.on_shutdown_triggered,
            callback_group=self.shutdown_callback_group,
            context=self.context
        )
        # real angle is 1.395 the magnitude of the input angle 
        self.declare_parameters(
            namespace='',
            parameters=[
            ('vel', 0.0),
            ('accel', 0.0),
            ('angle', 0.0),
            ('angular_vel', 0.0),
            ('verbose', False),
            ]
        )
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.cmd_vel_pub = self.create_publisher(AckermannDriveStamped, "cmd_vel", 10)
      
    def on_shutdown_triggered(self):
        self.get_logger().log("shutting down")
        
    def update_params(self):
        self.vel = self.get_parameter("vel").get_parameter_value().double_value
        self.accel = self.get_parameter("accel").get_parameter_value().double_value
        self.angle = self.get_parameter("angle").get_parameter_value().double_value
        self.angular_vel = self.get_parameter("angular_vel").get_parameter_value().double_value
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value  

    def timer_callback(self):
        self.update_params()
        
        ackerman_msg = AckermannDrive(
            steering_angle = self.angle,
            steering_angle_velocity = self.angular_vel,
            speed = self.vel,
            acceleration = self.accel,
            jerk = 0.0,
        )
        
        ackermannHeader = Header(stamp=self.get_clock().now().to_msg(),
                                  frame_id="gokart")
        
        msg = AckermannDriveStamped(
            header = ackermannHeader,
            drive = ackerman_msg
                )
        
        self.cmd_vel_pub.publish(msg)
        


from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.guard_condition import GuardCondition
import signal


def main(args=None):
    ctx = Context()
    
    def sigint_handler(signum, frame):
        node.get_logger().info("handling SIGINT - triggering shut down")
        node.shutdown_guard.trigger()
    
    signal.signal(signal.SIGINT, sigint_handler)
    rclpy.init(args=args, context=ctx, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    executor = MultiThreadedExecutor(context=ctx, num_threads=2)
    # executor = SingleThreadedExecutor(context=ctx)
    
    node = mock_stimulus_node(context=ctx)
    executor.add_node(node)
    

    try:
        executor.spin() # shuts down internally on SIGINT signal
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt signal intercepted")
    finally:
        node.get_logger().info("shutting down")

    node.destroy_node()

if __name__ == "__main__":
    main()
