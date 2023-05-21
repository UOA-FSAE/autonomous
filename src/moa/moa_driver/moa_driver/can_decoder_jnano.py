import rclpy
from rclpy.node import Node

# import all msg types needed
from moa_msgs.msg import CANStamped
from sensor_msgs.msg import BatteryState


class CANDecoderJNano(Node):
    def __init__(self) -> None:
        super().__init__("CAN_decoder")

        self.battery_count = 0
        self.battery_msg = BatteryState()

        # subscribe to raw can data from CAN_interface
        self.can_sub = self.create_subscription(
            CANStamped,
            "/raw_can",
            self.callback_can_data,
            10
        )

        # publish different status msgs
        self.battery_status_pub = self.create_publisher(BatteryState, "/battery_status", 10)
        self.steering_status_pub = self.create_publisher()
        self.steering_status_pub = self.create_publisher()

    def callback_can_data(self, msg: CANStamped):

        # it is assumed that different can msgs are published seperatly
        # however a better design strategy would be for it to be published as an array

        # switch case for different can ids
        # TODO make sure that the can id is accessed correctly
        # TODO make sure high byte + low byte done correctly

        # BATTERY STATUS

        if self.battery_count == 0:             # need to make sure all battery msgs are received
            self.battery_msg = BatteryState()
        
        if msg.can.id == 0x6b0:  
            self.battery_msg.present = True               
            self.battery_msg.current = int(msg.can.data[0] + msg.can.data[1])
            self.battery_msg.voltage = int(msg.can.data[2] + msg.can.data[3])
            self.battery_count += 1

        if msg.can.id == 0x6b3:
            self.battery_msg.cell_voltage = [float('nan') for _ in range(int(msg.can.data[6]))]
            self.battery_count += 1

        if msg.can.id == 0x6b4:
            self.battery_msg.temperature = int(msg.can.data[2] + msg.can.data[3])
            self.battery_count += 1

        if self.battery_count == 3:
            self.battery_status_pub.publish(self.battery_msg)
            self.battery_count = 0


        # STEERING STATUS


        # GLV STATUS


        # MOTOR STATUS


        # ERROR STATUS

def main(args=None):
    rclpy.init(args=args)

    CAN_decoder = CANDecoderJNano()

    rclpy.spin(CAN_decoder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    CAN_decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
