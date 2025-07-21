import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from rcl_interfaces.msg import SetParametersResult
from mission_control_lib.subscription_manager import MISSION_ANNOUNCE_TOPIC_NAME, MISSION_ANNOUNCE_QOS

"""
Authored By Chris (T0fu) Lobo

Missions indicated in a mission_array, with a 1 representing the topic will be subscribed to, while 0 means the topic is destroyed for that mission.
The missions are indexed as follows:
    0: "TESTING",
    1: "LAP 2 ONWARDS",
    2: "INSPECTION",
    3: "ACTUATOR CHECK",
    4: "MANUAL CONTROL",
    5: "AUTONOMOUS STATIC",
    6: "ACCELERATION",
    7: "SKIDPAD",
    8: "TRACKDRIVE",
    9: "EMERGENCY STOP",
    10: "MISSION COMPLETE"

The nodes will by default start in the TESTING mission. So ensure if you have any nodes using this library, that it be set to 1 in index 0
if you wish to test.

"""

class MissionControl(Node):
    def __init__(self):
        super().__init__('MissionControl')

        self.declare_parameter('current_mission', 0)
        self.current_mission = self.get_parameter('current_mission').get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self.on_param_change)

        self.mission_publisher = self.create_publisher(Int8, MISSION_ANNOUNCE_TOPIC_NAME, MISSION_ANNOUNCE_QOS)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'current_mission':
                self.current_mission = param.value
                self.publish_signals()
                self.get_logger().info(f"Mission changed to {param.value}")
        return SetParametersResult(successful=True)

    def publish_signals(self):
        msg = Int8()
        msg.data = self.current_mission
        self.mission_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()