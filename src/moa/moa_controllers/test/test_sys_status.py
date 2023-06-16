import pytest
import rclpy
from moa_controllers.sys_status import as_status


@pytest.fixture(autouse=True, scope='session')
def node():
    rclpy.init(args=None)
    test_status_node = as_status()

    yield test_status_node
    test_status_node.destroy_node()
    rclpy.shutdown()

def test_status(node: as_status):
    pass