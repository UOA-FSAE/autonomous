import rcply
from rcply.node import Node

from moa_msgs.msg import CANStamped
from moa_msgs.srv import CANSendReq

from .drivers.MCP2515 import MCP2515


class CANInterfaceJNano(Node):
    def __init__(self) -> None:
        super().__init__("CAN_interface")

        self.can = MCP2515()

        self.publish_can = self.create_publisher(
            CANStamped,
            "/raw_can",
            10
        )

        self.service_can = self.create_service(
            CANSendReq,
            'publish_can_data',
            self.callback_publish_can_data
        )

    def callback_publish_can_data(self, req, res):
        pass

    
def main():
    rcply.init()

    can_interface = CANInterfaceJNano()

    rcply.spin(can_interface)
    
    rcply.shutdown()

if __name__ == '__main__':
    main()
