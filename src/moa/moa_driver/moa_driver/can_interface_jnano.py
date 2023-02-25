import rcply
from rcply.node import Node

from moa_msgs.msg import CANStamped
from moa_msgs.srv import CANSendReq

from .drivers.MCP2515 import MCP2515


class CANInterfaceJNano(Node):
    def __init__(self) -> None:
        super().__init__("CAN_interface")

        self.can = MCP2515()
        self.can.Init()

        self.can_read_timer = self.create_timer(
            0.05,  # TODO: Change to veriable or const
            self.callback_read_can_data,
        )

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
        # TODO: need to probably rewrite the MCP2515 driver cuz there is litterly zero error checking
        self.can.Send(req.can.data, 8)

        res.sent = True
        return res

    def callback_read_can_data(self):
        raw_can_data = self.can.Receive()  # Don't know how this data is read out.
        can_data = raw_can_data  # This needs to be changed so that it turns the raw data into Stamed can msgs

        for can_frame in can_data:  # can_data is a list of can messages ready to be published
            self.publish_can.publish(can_frame)

    
def main(args=None):
    rcply.init(args=args)

    can_interface = CANInterfaceJNano()

    rcply.spin(can_interface)
    
    can_interface.destroy_node()
    rcply.shutdown()

if __name__ == '__main__':
    main()
