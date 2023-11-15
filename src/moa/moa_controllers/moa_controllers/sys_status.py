#!/usr/bin/env python3
# Python imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from re import search
from message_filters import ApproximateTimeSynchronizer, Subscriber

# ROS imports
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt8
from moa_msgs.msg import HardwareStatesStamped, MissionStatesStamped


class as_status(Node):
    def __init__(self):
        super().__init__('autonomous_sys_status')
        
        # init publishers
        self.status_pub = self.create_publisher(
            UInt8,
            'as_status',
            10,
        )
        
        # init subscribers
        self.ackermann_sub = Subscriber(
            self,
            AckermannDriveStamped,
            'moa/curr_vel',
        )

        self.mission_status_sub = Subscriber(
            self,
            MissionStatesStamped,
            'mission_status', # TODO change name
        )

        self.hardware_status_sub = Subscriber(
            self,
            HardwareStatesStamped,
            'moa/hardware_state', # TODO change name
        )
        
        # init Time Synchroniser
        self.time_sync = ApproximateTimeSynchronizer(
            [self.ackermann_sub, self.mission_status_sub, self.hardware_status_sub],
            queue_size = 5,
            slop=0.05,
        )

        self.time_sync.registerCallback(self.update_state)

        # from MoTec
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


    def check_mission_status(self, msg: MissionStatesStamped):
        """
            Records the states of hardware components 

            Args
            ------------------------------
            msg: A ROS message of type `MissionStateStamped`, containing the whether mission is selected or finished.
        """

        msg_data = msg.mission_states
        self.mission_selected = bool(msg_data.mission_selected)
        self.mission_finished = bool(msg_data.mission_finished)


    def check_hardware(self, msg: HardwareStatesStamped):
        """
            Records the states of hardware components 

            Args
            ------------------------------
            msg: A ROS message of type `HardwareStatesStamped`, containing the states of hardware components to be parsed.
        """

        msg_data = msg.hardware_states
        msg_attrs = [attr for attr in dir(msg_data) 
                    if not attr.startswith("_") and "serialize" not in attr]

        for attr in msg_attrs:
            if search("get_*", attr) is None and search("SLOT*", attr) is None:           
                self.hw_states[attr] = getattr(msg_data, attr)
        

    def check_car_stopped(self, msg: AckermannDriveStamped):
        """
            Checks whether car is stationary and records this information

            Args
            ------------------------------
            msg: A ROS message of type `AckermannDriveStamped`, containing the Ackermann drive commands to be parsed.
        """
        
        if msg.drive.speed == 0.0 and msg.drive.acceleration == 0.0:
            self.car_stopped = True
        else:
            self.car_stopped = False

    def update_state(self, ack_msg: AckermannDriveStamped =None, mission_msg: MissionStatesStamped = None, 
                     hw_msg: HardwareStatesStamped = None, prod: bool = True):
        """
            Updates the state of autonomous system using messages provided by time synchroniser. 
            
            Args
            ------------------------------
            - ack_msg: A ROS message of type `AckermannDriveStamped`, containing the Ackermann drive commands to be parsed.
            - mission_msg: A ROS message of type `MissionStateStamped`, containing the whether mission is selected or finished.
            - hw_msg: A ROS message of type `HardwareStatesStamped`, containing the states of hardware components to be parsed.
            - prod: boolean value signifying whether function is in production or testing mode

            Returns
            ------------------------------
            - Integer signifying the state of the Autonomous System.
            
            Possible AS states:
            0 : AS finished
            1 : AS emergency
            2 : AS ready
            3 : AS driving
            4 : AS off
        """
        if prod:
            self.check_car_stopped(ack_msg)
            self.check_hardware(hw_msg)
            self.check_mission_status(mission_msg)
        
        status_code = 4
        try:
            if self.hw_states['ebs_active']:
                if self.mission_finished and self.car_stopped:
                    status_code = 0
                else:
                    raise SystemExit
            else:
                if (self.mission_selected and self.hw_states['master_switch_on'] and 
                        self.hw_states['asb_ready'] and self.hw_states['ts_active']):
                    if self.hw_states['ts_active'] and self.hw_states['in_gear']:
                        status_code = 3
                    else:
                        if self.hw_states['brakes_engaged']:
                            status_code = 2       
        except SystemExit:
            self.get_logger().error("ERROR: AS in emergency state")
            status_code = 1
        finally:
            self.get_logger().info(f'status code: {status_code}')
            msg = UInt8(data=status_code)
            self.status_pub.publish(msg)
            return status_code


def main(args=None):
    rclpy.init(args=args)    
    node = as_status()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    
    except SystemExit:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()