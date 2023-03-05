#/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ctypes import c_uint8
import time

from ackermann_msgs import AckermannDriveStamped
from diagnostic_msgs import DiagnosticArray
from moa_msgs import CANSendReq

class ack_to_can(Node):
    def __init__(self):
        super().__init__('ackermann_to_can') #node name (NB: MoTec listens to this)

        #create subscriber for ackermann input
        self.subscription = self.create_subscription(
            AckermannDriveStamped,     #msg type
            'cmd_val',                 #topic receiving from
            self.listener_callback,    #callback function
            10                         #qos profile
        )

        #create publisher for diagnostics
        self.diagnostics = self.create_publisher(
            DiagnosticArray,           #msg type
            "diagnostics",             #topic sent to
            10                         #qos
        )
        
        #create client for heartbeat server
        self.heartbeat = self.create_client( #Unsure if needs to be server or client
            heartbeat_type,
            'heartbeat'
        )

        #create server for CAN
        self.can_client = self.create_client( #Unsure if needs to be server or client
            CANSendReq,
            'CAN_SEND_SRV'  
        )

        self.req = CANSendReq.Request()

    def listener_callback(self, msg): #TODO
        '''
        When receive message compute CAN message and submit async request to
        CAN_SEND_SRV
        '''
        
        input = msg.data
        pass
        
    def send_request(self,a): #TODO
        self.req.a = a
        self.future = self.can_client.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    node = ack_to_can()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
