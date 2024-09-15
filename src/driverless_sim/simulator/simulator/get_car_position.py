#!/usr/bin/python3
import rclpy
from rclpy.node import Node

<<<<<<< HEAD
from geometry_msgs.msg import Pose, Point
=======
from geometry_msgs.msg import Pose, Point, Quaternion
>>>>>>> origin/feat-new-track-msg-specification

import os
import sys
import matplotlib.pyplot as plt
import numpy as np

## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join('Formula-Student-Driverless-Simulator', 'python'))
sys.path.insert(0, path)
# sys.path.append('/home/Formula-Student-Driverless-Simulator/python')
# print(sys.path)
import fsds

class get_car_position(Node):
    def __init__(self):
        super().__init__("get_car_position")

        # connect to the simulator 
        self.client = fsds.FSDSClient(ip=os.environ['WSL_HOST_IP'])
        # Check network connection, exit if not connected
        self.client.confirmConnection()
        # After enabling setting trajectory setpoints via the api. 
        self.client.enableApiControl(True)
        # create publisher
        self.sim_car_pub = self.create_publisher(Pose, 'car_position', 10)

        self.create_timer(1.0, self.get_car_position)

    def get_car_position(self):
        """publishes current position of the simulator car"""
<<<<<<< HEAD
        position = self.client.getCarState().kinematics_estimated.position  # car kinetmatics in ENU coordinates
        
        msg = Pose()
        msg.position = Point(x=position.x_val, y=position.y_val, z=0.0) # add position information to Pose msg
=======
        state = self.client.getCarState()
        position = state.kinematics_estimated.position  # car kinetmatics in ENU coordinates
        orientation = state.kinematics_estimated.orientation
        
        msg = Pose()
        msg.position = Point(x=position.x_val, y=position.y_val, z=0.0) # add position information to Pose msg
        msg.orientation = Quaternion(x=orientation.x_val,y=orientation.y_val,z=orientation.z_val,w=orientation.w_val,)
>>>>>>> origin/feat-new-track-msg-specification

        self.sim_car_pub.publish(msg)   # publish msg

def main(args=None):
    rclpy.init(args=args)

    simulator_car_position = get_car_position()

    rclpy.spin(simulator_car_position)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator_car_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()