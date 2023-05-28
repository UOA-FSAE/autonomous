import rclpy
from rclpy.node import Node

# import all msg types needed
from moa_msgs.msg import CANStamped
from sensor_msgs.msg import BatteryState # https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html
from ackermann_msgs.msg import AckermannStamped


class CANDecoderJNano(Node):
    def __init__(self) -> None:
        super().__init__("CAN_decoder")

        self.battery_count = 0
        self.drive_count  = 0

        # subscribe to raw can data from CAN_interface
        self.can_sub = self.create_subscription(
            CANStamped,
            "/raw_can",
            self.callback_can_data,
            10
        )

        # publish different status msgs
        self.battery_status_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.drive_status_pub = self.create_publisher(AckermannStamped, "/drive_status", 10)
        self.steering_status_pub = self.create_publisher()

    def callback_can_data(self, msg: CANStamped):

        # it is assumed that different can msgs are published seperatly
        # however a better design strategy would be for it to be published as an array

        # switch case for different can ids
        # TODO make sure that the can id is accessed correctly
        # TODO make sure high byte + low byte done correctly

        # BATTERY STATUS

        # need to make sure all battery msgs are received before publishing
        if self.battery_count == 0 and msg.can.id == 0x6b0 or msg.can.id == 0x6b3 or msg.can.id == 0x6b4:             
            self.battery_msg = BatteryState()
            self.battery_count += 1
        
        if self.battery_count != 0: 
            if msg.can.id == 0x6b0:  
                self.battery_msg.present = True               
                self.battery_msg.current = int(msg.can.data[0] + msg.can.data[1])
                self.battery_msg.voltage = int(msg.can.data[2] + msg.can.data[3])
                self.battery_count += 1

            if msg.can.id == 0x6b3:
                self.battery_msg.cell_voltage = [float('nan') for _ in range(int(msg.can.data[6]))]
                self.battery_msg.cell_temperature = [float('nan') for _ in range(int(msg.can.data[6]))]
                self.battery_count += 1

            if msg.can.id == 0x6b4:
                self.battery_msg.temperature = int(msg.can.data[2] + msg.can.data[3])
                self.battery_count += 1

        if self.battery_count == 4:         # dont know which order it gets published
            self.battery_msg.header.stamp = self.get_clock().now().to_msg()         # not sure if this is needed
            self.battery_status_pub.publish(self.battery_msg)
            self.battery_count = 0


        # DRIVE STATUS
        if self.drive_count == 0 and msg.can.id == 0x604 or msg.can.id == 0x605:
            self.drive_status_msg = AckermannStamped()
            self.drive_count += 1

        if self.drive_count != 0:
            if msg.can.id == 0x604:
                self.drive_status_msg.drive.speed = int(msg.can.data[4])     # are we sure it is in binary
                self.drive_count += 1

            if msg.can.id == 0x605:
                self.drive_status_msg.drive.steering_angle = int(msg.can.data[0])
                self.drive_count += 1

        if self.drive_count == 3:
            self.drive_status_msg.header.stamp = self.get_clock().now().to_msg()         # not sure if this is needed
            self.drive_status_pub.publish(self.drive_status_msg)
            self.drive_count = 0

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
