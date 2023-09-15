#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose, PoseArray

class path_planning(Node):
    def __int__(self):
        super().__int__("path_planning")
        self.optimisation()

    # ===========================================================
    # TRAJECTORY GENERATION

    # ===========================================================
    # TRAJECTORY DELETION

    # ===========================================================
    # OPTIMISATION
    def optimisation(self):
        # get trajectories
        all_traj, all_states = self.generate_trajectories(5)
        # get best trajectory states
        opt_traj_state = self.get_best_trajectory_states(all_traj, all_states)

        return opt_traj_state

    def get_best_trajectory_states(self, trajectories, trajectories_states):
        # create an array of all trajectory lengths
        tlens = np.zeros(len(trajectories))
        # loop through each trajectory
        for i in range(len(trajectories)):
            # get length of trajectory i
            tlens[i] = self.get_trajectory_length(trajectories[i])
        # get state of largest trajectory
        states = self.get_trajectory_state(trajectories_states,np.argmax(tlens))

        return states

    def get_trajectory_length(self, trajectory: PoseArray):
        # take any two points
        p0 = [trajectory[0].position.x, trajectory[0].position.y]
        p1 = [trajectory[1].position.x, trajectory[1].position.y]
        # get linear distance between points
        intL = np.linalg.norm(p1-p0,ord=2)
        # get arc length
        arc_length = (len(trajectory)-1) * intL

        return arc_length

    def get_trajectory_state(self,trajectories_states,max_length):
        return trajectories_states[max_length]

    # FOR DEBUGGING
    def generate_trajectories(self, n):
        # list of all trajectories and states (for now just steering angle in rads)
        all_traj = []
        all_states = -np.random.random(n) + np.random.random(n)
        # make n pose arrays
        x = y = z = 0
        for i in range(n):
            pose_array = PoseArray()
            # make m poses with random coordinates
            for j in range(3):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = x, y, z
                pose_array[j] = pose
                # calculate new x, y, z sqrt((x2-x1)^2+(y2-y1)^2) = length with x2 unknown
                x = np.sqrt(0.1**2) + x
            # append pose array to all trajectories
            all_traj.append(pose_array)

        return all_traj, all_states

def main():
    rclpy.init()
    node = path_planning()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt as e:
        print(f"node spin error: {e}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()








