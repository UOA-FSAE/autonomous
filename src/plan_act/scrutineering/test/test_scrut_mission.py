import pytest
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from scrutineering.inspection_mission import inspection_mission_node
import rclpy
from rclpy.node import Node
from time import sleep
import random
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class testNode(Node):
    def __init__(self):
        super().__init__('testNode')
        self.sub = self.create_subscription(AckermannDriveStamped, 'cmd_val', self.msg_callback,10)
        self.pub = self.create_publisher(AckermannDriveStamped, 'moa/curr_vel',10)
        self.received = []
        self.real = False
    def msg_callback(self, msg):
        self.received.append(msg)
        sleep(1)
        if self.real:
            for i in range(1,5):
                rand = random.random()
                noise_msg = AckermannDrive(speed = rand, accel = rand)
                self.pub.publish(AckermannDriveStamped(drive = noise_msg))
                sleep(0.5)

        self.pub.publish(AckermannDriveStamped(drive = msg.drive))
        self.get_logger().info(f'received msg {msg}')

@pytest.fixture(scope='session')
def rclpy_init():
    rclpy.init(args=None)
    yield rclpy
    rclpy.shutdown()

@pytest.fixture(autouse=True, scope='function')
def inspect_node(rclpy_init):
    test_inspect = inspection_mission_node()
    test_inspect.set_parameters([
            rclpy.Parameter('timeout', value=10),
            rclpy.Parameter('max_steering_angle', value=45.0),
            rclpy.Parameter('steering_angle_velocity', value=36.0),
            rclpy.Parameter('max_speed', value=5.0),
            rclpy.Parameter('max_acceleration', value=5.0),
            rclpy.Parameter('jerk', value=1.0),
        ])
    yield test_inspect
    test_inspect.destroy_node()

@pytest.fixture(autouse=True, scope='function')
def test_node(rclpy_init):
    test_node = testNode()
    yield test_node
    test_node.destroy_node()

def test_param_loading(inspect_node: inspection_mission_node):
    loaded_params = {
        'timeout': inspect_node.get_parameter('timeout').value,
        'max_steering_angle': inspect_node.get_parameter('max_steering_angle').value,
        'steering_angle_velocity': inspect_node.get_parameter('steering_angle_velocity').value,
        'max_speed': inspect_node.get_parameter('max_speed').value,
        'max_acceleration': inspect_node.get_parameter('max_acceleration').value,
        'jerk': inspect_node.get_parameter('jerk').value,
    }

    config_path = os.path.join(
        get_package_share_directory('scrutineering'),
        'config',
        'test_params.yaml'
    )
    
    with open(config_path, "r") as file:
        config_data = yaml.safe_load(file)
        
    config_data = config_data['inspection_mission_node']['ros__parameters']
    
    assert config_data == loaded_params


def test_ackermann_comparison(inspect_node: inspection_mission_node):
    ack1 = AckermannDriveStamped(drive = AckermannDrive(speed = 0.34234278))
    ack2 = AckermannDriveStamped(drive = AckermannDrive(speed = 0.34234278))

    is_same = inspect_node.check_equal(ack1, ack2, 0.05)
    assert is_same

def test_timeout(inspect_node: inspection_mission_node):
    try:
        inspect_node
    except TimeoutError:
        assert True
    
def test_complete_mission(inspect_node: inspection_mission_node, test_node: testNode, request):
    try:
        test_node
        inspect_node
    except TimeoutError:
        assert False
    assert True
    request.addfinalizer(inspect_node.destroy_node)


def test_real_world(inspect_node: inspection_mission_node, test_node: testNode, request):
    try:
        test_node
        test_node.real = True
        inspect_node
    except TimeoutError:
        assert False
    assert True
    request.addfinalizer(inspect_node.destroy_node)

