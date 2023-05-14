import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from math import radians as to_rads
from rclpy.callback_groups import ReentrantCallbackGroup

class inspection_mission_node(Node):
    def __init__(self):
        super().__init__('inspection_mission_node')
        
        self.declare_parameters('', [
            ('timeout', 0),
            ('max_steering_angle', 0.0),
            ('steering_angle_velocity', 0.0),
            ('max_speed', 0.0),
            ('max_acceleration', 0.0),
            ('jerk', 0.0),
        ])
        
        self.publisher = self.create_publisher(
            AckermannDriveStamped, 
            'cmd_val', 
            10,
        )

        self.subscriber = self.create_subscription(
            AckermannDriveStamped,
            'moa/curr_vel',
            self.read_state,
            10,
        )
        
        self.curr_state = None

    def read_state(self, msg: AckermannDriveStamped): #subscriber callback
        self.get_logger().info('received msg')
        self.curr_state = msg

    def check_equal(self, sent: AckermannDriveStamped, received: AckermannDriveStamped, confidence: float) -> bool: 
        """
        Checks equality of 2 AckermannDriveStamped messages between a confidence interval

        Args
        ---------------------------
        - sent: The AckermannDriveStamped command sent to the car 
        - received: The AckermannDriveStamped message from moa/curr_vel depicting car's current state
        - confidence: Float value depicting percentage difference allowed for equality
        
        Returns
        ---------------------------
        - Boolean value saying whether the two AckermannDriveStamped messages are equal
        """
       
        is_same = False
        high, low = 1+confidence, 1-confidence
        if received is not None:
            sent = sent.drive
            received = received.drive
            is_same = True

            is_same = is_same and (abs(sent.steering_angle * low) <= abs(received.steering_angle) <= abs(sent.steering_angle * high))
            is_same = is_same and (abs(sent.steering_angle_velocity * low) <= abs(received.steering_angle_velocity) <= abs(sent.steering_angle_velocity * high))
            is_same = is_same and (abs(sent.speed * low) <= abs(received.speed) <= abs(sent.speed * high))
            is_same = is_same and (abs(sent.acceleration * low) <= abs(received.acceleration) <= abs(sent.acceleration * high))
            is_same = is_same and (abs(sent.jerk * low) <= abs(received.jerk) <= abs(sent.jerk * high))

        return is_same 

    def send_payloads(self, payloads: list[AckermannDrive]) -> None:
        """
        Takes a list of AckermannDrive messages and sends them to cmd_vel topic 
        and checks command executed before sending next message in list

        Args
        ---------------------------
        - payloads: list of AckermannDrive messages
        
        Returns
        ---------------------------
        - None
        
        If car doesn't respond to command in a certain time period TimeoutError is raised
        """
        
        timeout = self.get_parameter('timeout').value
        timeout = Duration(seconds=timeout)
        
        for data in payloads:
            msg = AckermannDriveStamped(drive = data)
            self.publisher.publish(msg)

            start_time = self.get_clock().now()
            end_time = start_time + timeout

            while (self.get_clock().now() < end_time) and rclpy.ok():
                rclpy.spin_once(self)
                is_same = self.check_equal(msg, self.curr_state, 0.05)
                if is_same:
                    self.get_logger().info("command complete")
                    break
                
    
            if not is_same:
                self.get_logger().error("timeout: command didn't complete")
                raise TimeoutError()

    def steer_test(self) -> None:
        """
        Executes steering test. Goes from -max_steering_angle to 0 to max_steering_angle to 0        
        """

        max_steer_angle = self.get_parameter('max_steering_angle').value
        steer_angle_vel = self.get_parameter('steering_angle_velocity').value
     
        payloads = [
            AckermannDrive(steering_angle=to_rads(-max_steer_angle), 
                           steering_angle_velocity=to_rads(steer_angle_vel)),
            AckermannDrive(steering_angle=to_rads(0.0), 
                           steering_angle_velocity=to_rads(steer_angle_vel)),
            AckermannDrive(steering_angle=to_rads(max_steer_angle), 
                           steering_angle_velocity=to_rads(steer_angle_vel)),
            AckermannDrive(steering_angle=to_rads(0.0), 
                           steering_angle_velocity=to_rads(steer_angle_vel))
            ]

        self.get_logger().info("commencing steering test")
        self.send_payloads(payloads)
        self.get_logger().info("complete steering test")
    
        

def main(args=None):
    rclpy.init(args=args)
    try:
        inspection_node = inspection_mission_node()

        executor = SingleThreadedExecutor()
        executor.add_node(inspection_node)
        
        executor.create_task(inspection_node.steer_test())

    finally:
        executor.shutdown()
        inspection_node.destroy_node()
  
        rclpy.shutdown()


if __name__ == '__main__':
    main()