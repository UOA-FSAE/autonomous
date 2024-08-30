#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from moa_msgs.msg import ConeMap

import os
import sys
import matplotlib.pyplot as plt
import numpy as np

## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../..','Formula-Student-Driverless-Simulator','python'))
sys.path.insert(0, path)
# sys.path.append('/home/Formula-Student-Driverless-Simulator/python')
# print(sys.path)
import fsds

class get_cones(Node):
    def __init__(self):
        super().__init__("get_cones")

        self.plot = True
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
        lb, rb, start_end = self.find_cones()  # detect cones in simulation

        msg = self.get_cone_map_msg(lb.copy(),rb.copy())   # convert the boundaries to point list

        self.sim_cone_pub.publish(msg)  # publish

        if self.plot:
            plt.show()
            while True:
                plt.pause(0.05)
                plt.clf()

                # get current position
                x,y,z = self.get_car_position()

                # plot
                plt.plot([P[0] for P in lb], [P[1] for P in lb], "*b", label="left")
                plt.plot([P[0] for P in rb], [P[1] for P in rb], "*y", label="right")
                plt.plot([P[0] for P in start_end], [P[1] for P in start_end], "*k", label='start/end')
                plt.plot(x,y,"*r",label="car")
                for i in range(len(lb)):
                    plt.annotate(f"{i}",(lb[i][0], lb[i][1]))
                for i in range(len(rb)):
                    plt.annotate(f"{i}",(rb[i][0], rb[i][1]))

                plt.legend()
                # plt.gca().invert_yaxis()
    

    def get_car_position(self):
        """returns current position of the simulator car"""
        position = self.client.getCarState().kinematics_estimated.position  # car kinetmatics in ENU coordinates
        
        return position.x_val, position.y_val, 0.0
    

    def find_cones(self):
        """ Detects cones in the simulator
        HOWEVER at the moment the exact positions of the cones is requested from API (change asap)"""
        # referee state
        ref_state = self.client.getRefereeState()

        cones = ref_state.cones  # list of cones where each cone is in a dictionary format e.g., [{cone1}, {cone2}, {cone3}]
        leftboundary = []
        rightboundary = []
        start_end = []

        for cone_dict in cones:
            color = cone_dict['color']
            x = cone_dict['x']
            y = cone_dict['y']
            z = 0.0 # z value is currently not provided 

            x,y,z = self.get_local_ENU([x,y,z]) # transform from UU to ENU coordinates 

            if color == 0:  # type 0 color cone is blue
                rightboundary.append([x,y,z])
            elif color == 1:    # type 1 color cone is yellow
                leftboundary.append([x,y,z])
            elif color == 2:    # orange start/end cones
                start_end.append([x,y,z])

        return leftboundary, rightboundary, start_end
    
    def get_local_ENU(self,position, world_to_meters=100):
        """Transform Unreal engine coordinates (UU) to AirSim coordiantes (ENU) - currently there is translation and scale (cm to m) I think not 100% sure
        For more details please see the below documents 
        1. https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/coordinate-frames/#unreal-engine
        2. https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/UE4Project/Plugins/AirSim/Source/CoordFrameTransformer.cpp
        """
        local_offset = np.array([4575.15,8577.82,0])    # translation vector from UU to ENU (fixed for now)
        return self.get_vector3r(position - local_offset, 1/world_to_meters)
    

    def get_vector3r(self, vec, scale):
        """Scales the given vector using the given real scalar"""
        return [vec[0]*scale, -vec[1]*scale, vec[2]*scale]


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


    # DEPRECATED/NOT USED 
    # def get_transformed_point(self, x, y):
    #     translation_matrix = np.array([
    #                 [1,0,4575.15],
    #                 [0,1,8577.81],
    #                 [0,0,1]
    #             ])
    #     rotation_matrix = np.array([
    #         [1,0,0],
    #         [0,-1,0],
    #         [0,0,-1]
    #     ])
    #     P = np.array([[x],[y],[1]])
    #     transformation_matrix = translation_matrix

    #     return np.linalg.inv(transformation_matrix) @ P


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