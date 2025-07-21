from rclpy.node import Node
from std_msgs.msg import Int8

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

MISSION_ANNOUNCE_TOPIC_NAME = "MissionSonic"
MISSION_ANNOUNCE_QOS = 10

class SubscriptionWrapper:
    def __init__(self, node, topic, msg_type, callback, mission_array, qos):
        self.node = node
        self.topic = topic
        self.msg_type = msg_type
        self.raw_callback = callback
        self.qos = qos
        self.control_sub = node.create_subscription(Int8, MISSION_ANNOUNCE_TOPIC_NAME, self.mission_change, MISSION_ANNOUNCE_QOS)
        self.mission_array = mission_array
        self.subscription = None

        if 0 in mission_array:
            self.subscription = self.node.create_subscription(self.msg_type, self.topic, self.raw_callback, self.qos)


    def mission_change(self, msg):
        if msg.data in self.mission_array and self.subscription is None:
            self.subscription = self.node.create_subscription(self.msg_type, self.topic, self.raw_callback, self.qos)
            # self.node.get_logger().info(f"Topic enabled for this node")
        elif msg.data not in self.mission_array and self.subscription is not None:
            try:
                self.node.destroy_subscription(self.subscription)
                self.subscription = None
                # self.node.get_logger().info(f"Topic disabled for this node")
            except:
                pass
        # else:
        #     self.node.get_logger().info(f"Topic state remains the same!")


def ManageSubscription(node: Node, topic, msg_type, callback, mission_array, qos_profile=10):
    return SubscriptionWrapper(node, topic, msg_type, callback, mission_array, qos_profile)
