import pytest
import rclpy
from moa_controllers.sys_status import as_status
from moa_msgs.msg import HardwareStatesStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt8


@pytest.fixture(autouse=True, scope='function')
def node():
    rclpy.init(args=None)
    
    test_status_node = as_status()
    test_status_node.update_timer = None

    yield test_status_node
    
    test_status_node.destroy_node()
    rclpy.shutdown()

def test_state_0(node: as_status):
    node.hw_states['ebs_active'] = True
    node.mission_finished = True
    node.car_stopped = True
    
    assert node.update_state() == 0


def test_state_1(node: as_status):
    node.hw_states['ebs_active'] = True
    test_cases = [[0,0],[0,1],[1,0]]

    for test in test_cases:
        node.mission_finished = test[0]
        node.car_stopped = test[1]
        
        assert node.update_state() == 1


def test_state_2(node: as_status):
    node.mission_selected = True
    node.hw_states['master_switch_on'] = True
    node.hw_states['asb_ready'] = True
    node.hw_states['ts_active'] = True
    
    node.hw_states['brakes_engaged'] = True
    
    assert node.update_state() == 2


def test_state_3(node: as_status):
    node.mission_selected = True
    node.hw_states['master_switch_on'] = True
    node.hw_states['asb_ready'] = True
    node.hw_states['ts_active'] = True
    
    node.hw_states['in_gear'] = True
    
    assert node.update_state() == 3


def test_state_4(node: as_status): # TODO to complete
    node.mission_selected = True
    node.hw_states['master_switch_on'] = True
    node.hw_states['asb_ready'] = True
    node.hw_states['ts_active'] = True

    assert node.update_state() == 4


def test_check_mission_status(node: as_status): # TODO need to sort out planning/complete
    msg = UInt8(data = 0) 
    node.check_mission_status(msg)
    assert node.mission_selected == False and node.mission_finished == True

    msg = UInt8(data = 1) 
    node.check_mission_status(msg)
    assert node.mission_selected == False and node.mission_finished == False

    msg = UInt8(data = 2) 
    node.check_mission_status(msg)
    assert node.mission_selected == True and node.mission_finished == True


def test_check_car_stopped(node: as_status):
    msg = AckermannDriveStamped()
    
    msg.drive.speed = 0.0
    msg.drive.acceleration = 0.0
    node.check_car_stopped(msg)
    assert node.car_stopped

    msg.drive.speed = 1.0
    msg.drive.acceleration = 1.0
    node.check_car_stopped(msg)
    assert not node.car_stopped


def test_check_hardware(node: as_status): # TODO complete
    msg = HardwareStatesStamped()
    msg.hardware_states.ebs_active = 1
    msg.hardware_states.ts_active = 0
    msg.hardware_states.in_gear = 1
    msg.hardware_states.master_switch_on = 1
    msg.hardware_states.brakes_engaged = 1

    test_case = {
        'ebs_active': 1,
        'ts_active': 0,
        'in_gear': 1,
        'master_switch_on': 1,
        'asb_ready': 0,
        'brakes_engaged': 1,
    }

    node.check_hardware(msg)
    assert node.hw_states == test_case