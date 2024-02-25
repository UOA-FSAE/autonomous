import rclpy
from rclpy.node import Node
from moa_msgs.msg import Pulse
from std_msgs.msg import UInt8, String

MAXLAPS = 3

START_DESCRIPTION = ""
END_DESCRIPTION = ""

class EventController(Node):
    def __init__(self):
        super().__init__('event_controller')

        self.start_end_subsc = self.create_subscription(
            Pulse,
            'start_end_detection',
            self.pulse_callback,
            10)
        self.at_start = False
        self.at_finish = False

        self.round = 0

        self.assi_finished_pub = self.create_publisher(
            Pulse,
            'assi_mission_finished',
            10,
        )


        self.start_desc_pub = self.create_publisher(
            String,
            'assi_mission_finished',
            10,
        )

        self.end_desc_pub = self.create_publisher(
            String,
            'assi_mission_finished',
            10,
        )

        self.timer1 = self.create_timer(1, self.publish_start_description)
        self.timer2 = self.create_timer(1, self.publish_end_description)

    def publish_start_description(self):
        start_desc_msg = String()
        start_desc_msg.data = START_DESCRIPTION
        self.start_desc_pub.publish(start_desc_msg)
    
    def publish_end_description(self):
        end_desc_msg = String()
        end_desc_msg.data = END_DESCRIPTION
        self.end_desc_pub.publish(end_desc_msg)

    def pulse_callback(self, msg):
        if msg.value:
            msgToSend = Pulse()
            msgToSend.value = False
            # If pulse is True, it means the event has occurred
            if msg.event_type == 'start':
                self.at_start = True
                self.at_finish = False

            elif msg.event_type == 'finish':
                if self.round < MAXLAPS:
                    self.round += 1
                else:
                    self.at_finish = True
                    self.at_start = False
                    msgToSend.value = True

            self.self.assi_finished_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    self_driving_car = SelfDrivingCar()
    rclpy.spin(self_driving_car)
    self_driving_car.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
