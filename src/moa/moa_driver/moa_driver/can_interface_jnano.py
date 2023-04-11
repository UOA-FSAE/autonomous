import rcply
from rcply.node import Node

from moa_msgs.msg import CANStamped
from moa_msgs.srv import CANSendReq

from .drivers.MCP2515 import MCP2515

# TIMER_

class CANInterfaceJNano(Node):
    def __init__(self) -> None:
        super().__init__("CAN_interface")

        self.can = MCP2515()
        self.can.Init()

        self.can_read_timer = self.create_timer(
            0.05,  # TODO: Change to variable or const
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

        # Checking to see if a can message is valid to send
        # List of things that need checking
        # * Is CAN ID able to fit in 11 bits 
        # * If any data value is not a uint8_t
        if req.can.id > 2047:
            self.get_logger().error(f"Publish request for CAN failed: CAN_ID to large, {req.can.id} > 2047 / 11 bits")

            res.sent = False
            return res
        elif min(req.can.data) < 0b0 or max(req.can.data) > 0b11111111:
            self.get_logger().error(f"Publish request for CAN failed: Data value is not uint8_t, {req.can.data=}")

            res.sent = False
            return res

        # TODO: Need to full rewrite can driver as we can not set id or rtr of the can msg as well as no error
        #  checking :(
        try:
            self.can.Send(req.can.data, 8)  # TODO:This needs to be changed to include the id and is_rtr

        # TODO: this needs to be changed so that it catches specific errors and not just all of them.
        except Exception as e:
            self.get_logger().error(f"Publish request for CAN failed: Failed to send message for CAN msg, {req.can=}")
            self.get_logger().error(f"Error msg: {e}")

            res.sent = False
            return res

        res.sent = True
        return res

    def callback_read_can_data(self):
        raw_can_data = self.can.Receive()  # Don't know how this data is read out.
        can_data = raw_can_data  # This needs to be changed so that it turns the raw data into Stamped can msgs

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
