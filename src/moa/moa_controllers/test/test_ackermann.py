import pytest
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from moa_controllers.ack_to_can import ack_to_can
from unittest.mock import Mock
import rclpy



@pytest.fixture(autouse=True, scope='module')
def node():
    rclpy.init(args=None)
    test_ack = ack_to_can()

    yield test_ack
    test_ack.destroy_node()
    rclpy.shutdown()


def test_ackermann_to_can_parser_out_of_bounds(node: ack_to_can):
    ack_msg = AckermannDriveStamped()
    
    #Check the conditions for the ackermann out of bounds test (think need to change the inequalities)

    ack_msg.drive.speed = 130/3.6
    temp = node.ackermann_to_can_parser(ack_msg)
    assert (node.ackermann_to_can_parser(ack_msg) is None) #speed

    ack_msg.drive.speed = 0.0
    ack_msg.drive.acceleration = 257.0
    assert (node.ackermann_to_can_parser(ack_msg) is None) #acceleration

    ack_msg.drive.acceleration = 0.0
    ack_msg.drive.jerk = -1.0
    assert node.ackermann_to_can_parser(ack_msg) is None #jerk

    ack_msg.drive.jerk = 0.0
    ack_msg.drive.steering_angle = 50*np.pi/180
    assert node.ackermann_to_can_parser(ack_msg) is None #steering angle

    ack_msg.drive.steering_angle = 0.0
    ack_msg.drive.steering_angle_velocity = -1.5
    assert node.ackermann_to_can_parser(ack_msg) is None #steering angle


def test_ackermann_to_can_parser_valid_values(node: ack_to_can):
    ack_msg = AckermannDriveStamped()
    ack_msg.drive.speed = 20/3.6 # m/s
    ack_msg.drive.acceleration = 128.0 # m/s^2
    ack_msg.drive.jerk = 0.001 # m/s^3
    ack_msg.drive.steering_angle = np.pi/4 # radians
    ack_msg.drive.steering_angle_velocity = 0.5 # rad/s
    
    steering_angle = np.float16(ack_msg.drive.steering_angle).tobytes()
    s_a_size = len(steering_angle)

    expected_result = np.array([
        ack_msg.drive.speed,
        ack_msg.drive.acceleration,
        ack_msg.drive.jerk*100,
        int.from_bytes(steering_angle[:s_a_size//2], 'big'),
        int.from_bytes(steering_angle[s_a_size//2:], 'big'),
        ack_msg.drive.steering_angle_velocity*100,
        0,
        0], dtype=np.uint8)
    
    can_data = node.ackermann_to_can_parser(ack_msg) 
    assert np.all(can_data == expected_result)


def test_ack_to_can_publish_callback(node: ack_to_can):
    # Create a mock CAN publisher and inject it into the node
    mock_publisher = Mock()
    node.can_pub = mock_publisher

    # Create an example AckermannDriveStamped message
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp.sec = 1
    ack_msg.header.stamp.nanosec = 500000000
    ack_msg.drive.speed = 10.0
    ack_msg.drive.acceleration = 100.0
    ack_msg.drive.jerk = 0.1
    ack_msg.drive.steering_angle = 0.5
    ack_msg.drive.steering_angle_velocity = 0.8

    # Call the method under test
    node.ack_to_can_publish_callback(ack_msg)

    # Check that the CAN publisher was called once
    mock_publisher.publish.assert_called_once()

    # Check that the published message has the correct ID and data
    args, kwargs = mock_publisher.publish.call_args
    published_msg = args[0]

    
    steering_angle = np.float16(ack_msg.drive.steering_angle).tobytes()
    s_a_size = len(steering_angle)

    expected_result = np.array([
        ack_msg.drive.speed,
        ack_msg.drive.acceleration,
        ack_msg.drive.jerk*100,
        int.from_bytes(steering_angle[:s_a_size//2], 'big'),
        int.from_bytes(steering_angle[s_a_size//2:], 'big'),
        ack_msg.drive.steering_angle_velocity*100,
        0,
        0], dtype=np.uint8)

    assert published_msg.can.id == 25
    assert np.all(published_msg.can.data == expected_result)