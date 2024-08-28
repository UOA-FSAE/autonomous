#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import os
import sys
import matplotlib.pyplot as plt
## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..','Formula-Student-Driverless-Simulator','python'))
sys.path.insert(0, path)
# sys.path.append('/home/Formula-Student-Driverless-Simulator/python')
# print(sys.path)
import fsds

class get_cones(Node):
    def __init__(self):
        super().__init__("get_cones")

        # connect to the simulator 
        self.client = fsds.FSDSClient(ip=os.environ['WSL_HOST_IP'])
        # Check network connection, exit if not connected
        self.client.confirmConnection()
        # After enabling setting trajectory setpoints via the api. 
        self.client.enableApiControl(True)

        self.count = 0
        self.get_cones_from_simulator()

    def get_cones_from_simulator(self):
        lb, rb = self.find_cones()

        plt.plot([P[0] for P in lb], [P[1] for P in lb], "*b")
        plt.plot([P[0] for P in rb], [P[1] for P in rb], "*y")
        plt.show()

    
    def find_cones(self):
        """ gets the exact position of the cones using the API - cheating as not sensor information used"""
        # referee state
        ref_state = self.client.getRefereeState()

        cones = ref_state.cones  # list of cones where each cone is in a dictionary format e.g., [{cone1}, {cone2}, {cone3}]
        leftboundary = []
        rightboundary = []

        for cone_dict in cones:
            color = cone_dict['color']
            x = cone_dict['x']
            y = cone_dict['y']

            if color == 0:
                leftboundary.append([x,y])
            elif color == 1:
                rightboundary.append([x,y])

        return leftboundary, rightboundary


def main(args=None):
    rclpy.init(args=args)

    simulator_cones = get_cones()

    rclpy.spin(simulator_cones)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator_cones.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()