import pytest
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from scrutineering.scrut_mission import scrut_mission_node
import rclpy
from rclpy.node import Node
from time import sleep

class testNode(Node):
        def __init__(self):
            super().__init__('testNode')
            self.sub = self.create_subscription(AckermannDriveStamped, 'cmd_val', self.msg_callback,10)
            self.pub = self.create_publisher(AckermannDriveStamped, 'moa/curr_vel',10)
            self.received = []
        def msg_callback(self, msg):
            self.received.append(msg)
            sleep(1)
            self.pub.publish(AckermannDriveStamped(drive = msg.drive))
            self.get_logger().info(f'received msg {msg}')

@pytest.fixture(scope='session')
def rclpy_init():
    rclpy.init(args=None)
    yield rclpy
    rclpy.shutdown()

@pytest.fixture(autouse=True, scope='function')
def scrut_node(rclpy_init):
    test_scrut = scrut_mission_node()
    yield test_scrut
    test_scrut.destroy_node()

@pytest.fixture(autouse=True, scope='function')
def test_node(rclpy_init):
    test_node = testNode()
    yield test_node
    test_node.destroy_node()
    
def test_ackermann_comparison(scrut_node: scrut_mission_node):
    ack1 = AckermannDriveStamped(drive = AckermannDrive(speed = 0.34234278))
    ack2 = AckermannDriveStamped(drive = AckermannDrive(speed = 0.34234278))

    is_same = scrut_node.check_equal(ack1, ack2, 0.05)
    assert is_same

def test_timeout(scrut_node: scrut_mission_node):
    try:
        scrut_node
    except TimeoutError:
        assert True
    
def test_send_messages(scrut_node: scrut_mission_node, test_node):
    try:
        test_node
        scrut_node
    except TimeoutError:
        assert False
    assert True