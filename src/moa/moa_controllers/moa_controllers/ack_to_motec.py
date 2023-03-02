#/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs import AckermannDriveStamped
from diagnostic_msgs import DiagnosticStatus

class ack_to_mot(Node):
    def __init__(self):
        super().__init__('ackermann_to_motec') #node name

        #create subscriber for ackermann input
        self.subscription = self.create_subscription(
            AckermannDriveStamped,     #msg type
            'cmd_val',                 #topic receiving from
            self.listener_callback,    #callback function
            10                         #qos profile
        )

        #create publisher for diagnostics
        self.diagnostics = self.create_publisher(
            DiagnosticStatus,          #msg type
            "diagnostics",             #topic sent to
            10                         #qos
        )
        
        # #create client for heartbeat server
        # self.heartbeat = self.create_client(
            
        # )

        # #create server for CAN
        # self.server = self.create_service(

        # )


    def listener_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = ack_to_mot()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
