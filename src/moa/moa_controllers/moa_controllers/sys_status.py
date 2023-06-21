#!/usr/bin/env python3
# Python imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from re import search

# ROS imports
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt8
from moa_msgs.msg import HardwareStatesStamped

class as_status(Node):
    def __init__(self):
        super().__init__('autonomous_sys_status')
        
        self.sub_cbgroup = ReentrantCallbackGroup()

        # publishers
        self.status_pub = self.create_publisher(
            UInt8,
            'as_status',
            10,
        )
        
        # subscribers
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            'moa/curr_vel',
            self.check_car_stopped,
            10,
            callback_group = self.sub_cbgroup,
        )

        self.mission_status_sub = self.create_subscription(
            UInt8,
            'as_status', # TODO change name
            self.check_mission_status,
            10,
            callback_group = self.sub_cbgroup,
        )

        self.hardware_status_sub = self.create_subscription(
            HardwareStatesStamped,
            'moa/hardware_state', # TODO change name
            self.check_hardware,
            10,
            callback_group = self.sub_cbgroup,
        )
        
        #timer
        self.update_timer = self.create_timer(
            1/50, #time in seconds (currently equiv of 50 times per second)
            self.update_state,
        )

        # system states
        self.hw_states = {
            'ebs_active': False,
            'ts_active': False,
            'in_gear': False,
            'master_switch_on': False,
            'asb_ready': False,
            'brakes_engaged': False,
            }
        
        # from moa/curr_vel
        self.car_stopped = False      

        # from event controller
        self.mission_finished = False
        self.mission_selected = False


    def check_mission_status(self, msg: UInt8): # TODO needs more planning work done
        if msg.data == 0:
            self.mission_finished = True            
        
        elif msg.data == 1:
            self.mission_selected = False
            self.mission_finished = False

        elif msg.data == 2:
            self.mission_selected = True
        
        self.get_logger().info('checked mission status')
            

    def check_hardware(self, msg: HardwareStatesStamped): #TODO
        msg_data = msg.hardware_states
        msg_attrs = [attr for attr in dir(msg_data) 
                    if not attr.startswith("_") and "serialize" not in attr]

        for attr in msg_attrs:
            if search("get_*", attr) is None and search("SLOT*", attr) is None:           
                self.hw_states[attr] = getattr(msg_data, attr)
        
        self.get_logger().info('checked hardware')
        

    def check_car_stopped(self, msg: AckermannDriveStamped):
        if msg.drive.speed == 0.0 and msg.drive.acceleration == 0.0:
            self.car_stopped = True
        else:
            self.car_stopped = False
        
        self.get_logger().info('checked car motion')


    def update_state(self):
        '''
            AS states:
            0 : AS finished
            1 : AS emergency
            2 : AS ready
            3 : AS driving
            4 : AS off
        '''
        status_code = 4
        try:
            if self.hw_states['ebs_active']:
                if self.mission_finished and self.car_stopped:
                    status_code = 0
                else:
                    raise Exception
            else:
                if (self.mission_selected and self.hw_states['master_switch_on'] and 
                        self.hw_states['asb_ready'] and self.hw_states['ts_active']):
                    if self.hw_states['ts_active'] and self.hw_states['in_gear']:
                        status_code = 3
                    else:
                        if self.hw_states['brakes_engaged']:
                            status_code = 2       
        except Exception:
            self.get_logger().error("ERROR: AS in emergency state")
            status_code = 1
        finally:
            msg = UInt8(data=status_code)
            self.status_pub.publish(msg)
            return status_code


def main(args=None):
    rclpy.init(args=args)
    
    node = as_status()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
