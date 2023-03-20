#/usr/bin/env python3
import pytest
import rclpy

@pytest.fixture(scope='module')
def ros_node():
    rclpy.init()
    node = rclpy.create_node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_node_functionality(ros_node):
    # test the functionality of the ROS2 node
    assert 