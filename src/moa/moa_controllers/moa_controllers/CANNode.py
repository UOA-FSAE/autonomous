from .CANdapter import CANDapter, CANFrame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CANNode(Node):
    def __init__(self):
        super().__init__('CANNode')

        self.curr_throtle = '00'
        self.curr_steering_angle = '64' # 100 in hex so that it sets the defult steering position to stright

        self.can_dapter = CANDapter(port="/dev/ttyUSB0")

        self.can_dapter.close_channel()
        self.can_dapter.set_bitrate(250)
        self.can_dapter.open_channel()

        self.throttle_topic = self.create_subscription(Float32, 'set_throtle', self.throttle_cb, 10)
        self.steering_topic = self.create_subscription(Float32, 'set_steering', self.steering_cb, 10)

    def throttle_cb(self, msg):
        self.curr_throtle = hex(int(msg.data))[2:]
        if len(self.curr_throtle) != 2:
            self.curr_throtle = "0" + self.curr_throtle
            
        can_msg = CANFrame(250, 2, [self.curr_throtle, self.curr_steering_angle])
        self.can_dapter.send_can_message(can_msg)

    def steering_cb(self, msg):
        angle = min(max(msg.data, -100.0), 100)
        self.curr_steering_angle = hex(int((angle + 100)))[2:]
        print(self.curr_steering_angle)
        if len(self.curr_throtle) != 2:
            self.curr_throtle = "0" + self.curr_throtle

        can_msg = CANFrame(250, 2, [self.curr_throtle, self.curr_steering_angle])
        self.can_dapter.send_can_message(can_msg)


def main():
    rclpy.init()

    can_node = CANNode()

    rclpy.spin(can_node)



if __name__ == '__main__':
    main()  