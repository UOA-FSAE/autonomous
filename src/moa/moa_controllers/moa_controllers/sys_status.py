#!/usr/bin/env python3
# Python imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

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
            '',
            self.check_mission_status,
            10,
            callback_group = self.sub_cbgroup,
        )

        self.hardware_status_sub = self.create_subscription(
            HardwareStatesStamped,
            '',
            self.check_hardware,
            10,
            callback_group = self.sub_cbgroup,
        )
        
        #timer
        self.update_timer = self.create_timer(
            1/50, #time in seconds (currently equiv of 50Hz)
            self.update_state,
        )

        # system states
        # from motec
        self.ebs_state = False
        self.ts_active = False
        self.in_gear = False
        self.master_switch_on = False
        self.asb_ready = False
        self.brakes_engaged = False
        
        # from moa/curr_vel
        self.car_stopped = False      

        # from event controller
        self.mission_finished = False
        self.mission_selected = False


    def check_mission_status(self, msg: UInt8): # TODO needs more planning work done
        if msg.data == 0:
            self.mission_selected = False
            self.mission_finished = False
        
        if msg.data == 1:
            self.mission_finished == True
        elif msg.data == 2:
            self.mission_selected = True
        
        self.get_logger().info('checked mission status')
            

    def check_hardware(self, msg: HardwareStatesStamped):
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
            if self.ebs_state:
                if self.mission_status and self.car_stopped:
                    status_code = 0
                else:
                    raise Exception
            else:
                if self.mission_selected and self.master_switch_on and self.asb_ready and self.ts_active:
                    if self.ts_active and self.in_gear:
                        status_code = 3
                    else:
                        if self.brakes_engaged:
                            status_code = 2       
        except:
            status_code = 1
        
        msg = UInt8(data=status_code)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = as_status()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
