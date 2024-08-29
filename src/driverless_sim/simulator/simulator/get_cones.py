#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from moa_msgs.msg import ConeMap

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

        self.plot = False
        # connect to the simulator 
        self.client = fsds.FSDSClient(ip=os.environ['WSL_HOST_IP'])
        # Check network connection, exit if not connected
        self.client.confirmConnection()
        # After enabling setting trajectory setpoints via the api. 
        self.client.enableApiControl(True)
        # create publisher
        self.sim_cone_pub = self.create_publisher(ConeMap, "cone_map", 10)

        self.get_cones_from_simulator()


    def get_cones_from_simulator(self):
        lb, rb = self.find_cones()  # detect cones in simulation

        msg = self.get_cone_map_msg(lb.copy(),rb.copy())   # convert the boundaries to point list

        self.sim_cone_pub.publish(msg)  # publish

        if self.plot:
            plt.plot([P[0] for P in lb], [P[1] for P in lb], "*b", label="left")
            plt.plot([P[0] for P in rb], [P[1] for P in rb], "*y", label="right")
            for i in range(len(lb)):
                plt.annotate(f"{i}",(lb[i][0], lb[i][1]))
            for i in range(len(rb)):
                plt.annotate(f"{i}",(rb[i][0], rb[i][1]))
            plt.legend()
            plt.show()
    
    def find_cones(self):
        """ Detects cones in the simulator
        HOWEVER at the moment the exact positions of the cones is requested from API (change asap)"""
        # referee state
        ref_state = self.client.getRefereeState()

        cones = ref_state.cones  # list of cones where each cone is in a dictionary format e.g., [{cone1}, {cone2}, {cone3}]
        leftboundary = []
        rightboundary = []

        for cone_dict in cones:
            color = cone_dict['color']
            x = cone_dict['x']
            y = cone_dict['y']

            if color == 0:  # type 0 color cone is blue
                leftboundary.append([x,y,0.0])
            elif color == 1:    # type 1 color cone is yellow
                rightboundary.append([x,y,0.0])

        return leftboundary, rightboundary

    def get_cone_map_msg(self, lb, rb):
        try:
            assert len(lb) == len(rb)
        except Exception as e:
            self.get_logger().error(e)
        finally:
            for i in range(len(lb)):
                lb[i] = Point(x=lb[i][0],y=lb[i][1],z=lb[i][2])
                rb[i] = Point(x=rb[i][0],y=rb[i][1],z=rb[i][2])

        return ConeMap(left_cones=lb, right_cones=rb)

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