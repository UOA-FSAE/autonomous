#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

class path_planning(Node):
    def __init__(self):
        super().__init__("path_planning")
        self.optimisation()

    # ===========================================================
    # TRAJECTORY GENERATION
    def base_trajectory_generator(self, steering_angle):
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357 is used for
        # bicycle steering
        L = 1;
        R = L / np.tan(steering_angle);
        t_range = np.arange(0, np.pi/2, 0.01);
        trajectory_output = PoseArray();
        for individual_t in t_range:
            pose_input = Pose();
            pose_input.Point.x = np.cos(individual_t) * R
            pose_input.Point.y = np.sin(individual_t) * R
            trajectory_output.poses.append(pose_input)
        return trajectory_output

    def trajectory_generator(self):
        candidate_steering_angle = np.deg2rad(np.arange(-10, 10, 0.5))
        trajectories = []
        for steering_angle in candidate_steering_angle:
            added_trajectory = self.base_trajectory_generator(steering_angle)
            trajectories.append(added_trajectory)
        return trajectories
    def get_current_position(self):
        pass;

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
        poses = trajectory.poses
        p0 = np.array([poses[0].position.x, poses[0].position.y])
        p1 = np.array([poses[1].position.x, poses[1].position.y])
        # get linear distance between points
        intL = np.linalg.norm(p1-p0,ord=2)
        # get arc length
        arc_length = (len(poses)-1) * intL

        return arc_length

    def get_trajectory_state(self,trajectories_states,max_length):
        return trajectories_states[max_length]

    # FOR DEBUGGING
    def generate_trajectories(self, n):
        # list of all trajectories and states (for now just steering angle in rads)
        all_traj = []
        all_states = -np.random.random(n) + np.random.random(n)
        # make n pose arrays
        x = y = z = 0.0
        for i in range(n):
            # contains poses for ith pose array
            temp = []
            # make m poses with random coordinates
            for j in range(3):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = x, y, z
                # pose_array.poses = pose
                temp.append(pose)
                # calculate new x, y, z sqrt((x2-x1)^2+(y2-y1)^2) = length with x2 unknown
                x = np.sqrt(0.1**2) + x
            pose_array = PoseArray()
            pose_array.poses = temp
            # append pose array to all trajectories
            all_traj.append(pose_array)

        return all_traj, all_states

def main():
    rclpy.init()
    service = path_planning()
    try:
        rclpy.spin_once(service)
    except Exception as e:
        print(f"node spin error: {e}")
    service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()








