import pytest
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from moa_msgs.msg import CANStamped
from moa_controllers.ack_to_can import ack_to_can
from unittest.mock import Mock
import rclpy



@pytest.fixture(autouse=True, scope='session')
def node():
    rclpy.init(args=None)
    test_ack = ack_to_can()

    yield test_ack
    test_ack.destroy_node()
    rclpy.shutdown()

    
# def test_compress_floats():
#     values = np.array([1.0, 1.5, 2.0, 2.5])
#     expected_result = b'x\x9c+\x01\x00\xf6\xff,\x02\x00\xfa\xff'
#     assert compress_floats(values) == expected_result


def test_ackermann_to_can_parser_out_of_bounds(node: ack_to_can):
    ack_msg = AckermannDriveStamped()
    
    ack_msg.drive.speed = 130/3.6
    assert node.ackermann_to_can_parser(ack_msg) is None

    ack_msg.drive.speed = 0
    ack_msg.drive.acceleration = 257
    assert node.ackermann_to_can_parser(ack_msg) is None

    ack_msg.drive.acceleration = 0
    ack_msg.drive.jerk = -1
    assert node.ackermann_to_can_parser(ack_msg) is None

    ack_msg.drive.jerk = 0
    ack_msg.drive.steering_angle = 50*np.pi/180
    assert node.ackermann_to_can_parser(ack_msg) is None

    ack_msg.drive.steering_angle = 0
    ack_msg.drive.steering_angle_velocity = -1.5
    assert node.ackermann_to_can_parser(ack_msg) is None


def test_ackermann_to_can_parser_valid_values(node: ack_to_can):
    ack_msg = AckermannDriveStamped()
    ack_msg.drive.speed = 20/3.6 # m/s
    ack_msg.drive.acceleration = 128 # m/s^2
    ack_msg.drive.jerk = 0.001 # m/s^3
    ack_msg.drive.steering_angle = np.pi/4 # radians
    ack_msg.drive.steering_angle_velocity = 0.5 # rad/s
    
    expected_result = [
        np.uint8(round(ack_msg.drive.speed)),
        np.uint8(round(ack_msg.drive.acceleration)),
        np.uint8(ack_msg.drive.jerk*1000),
        np.float16(ack_msg.drive.steering_angle),
        np.uint8(ack_msg.drive.steering_angle_velocity*1000),
    ]
    
    can_data = node.ackermann_to_can_parser(ack_msg) 
    assert can_data == expected_result


def test_ack_to_can_publish_callback(node: ack_to_can):
    # Create a mock CAN publisher and inject it into the node
    mock_publisher = Mock()
    node.can_pub = mock_publisher

    # Create an example AckermannDriveStamped message
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp.sec = 1
    ack_msg.header.stamp.nanosec = 500000000
    ack_msg.drive.speed = 10
    ack_msg.drive.acceleration = 100
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

    assert published_msg.can.id == 56
    assert published_msg.can.data == [10, 100, 100, 59, 204, 0]