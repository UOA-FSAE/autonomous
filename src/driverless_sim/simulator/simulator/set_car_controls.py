#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive

import os
import sys
import numpy as np

## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join('Formula-Student-Driverless-Simulator', 'python'))
sys.path.insert(0, path)
# sys.path.append('/home/Formula-Student-Driverless-Simulator/python')
# print(sys.path)
import fsds

class set_car_controls(Node):
    def __init__(self):
        super().__init__("set_car_controls")

        self.max_throttle = 21  # m/s
        self.max_steering = 25  # degrees
        # connect to the simulator 
        self.client = fsds.FSDSClient(ip=os.environ['WSL_HOST_IP'])
        # Check network connection, exit if not connected
        self.client.confirmConnection()
        # After enabling setting trajectory setpoints via the api. 
        self.client.enableApiControl(True)
        # create publisher
        self.create_subscription(AckermannDrive, 'drive', self.callback, 10)


    def callback(self, msg:AckermannDrive):
        """Set the car controls in the simulator using the given ackerman msg"""
        steering = msg.steering_angle
        speed = msg.speed

        steering, throttle = self.get_simulator_controls(steering, speed)

        # class is part of types.py file in sim repo
        self.client.setCarControls(fsds.CarControls(throttle=throttle,steering=steering,brake=0.0))

    def get_simulator_controls(self, steering, speed):
        throttle = speed/self.max_throttle # max throttle is 1 in sim where the max speed is ~20m/s
        steering = steering/self.max_steering # max absolute steering is 1 in sim where the max steering angle is 25 degrees by default

        return steering, throttle


def main(args=None):
    rclpy.init(args=args)

    simulator_car_controls = set_car_controls()

    rclpy.spin(simulator_car_controls)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator_car_controls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()