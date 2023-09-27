#!/usr/bin/python3
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from mapping_interfaces.msg import Cone, ConeMap

class path_planning(Node):
    def __init__(self):
        super().__init__("path_planning")
        # TANISH - DANIEL you can change the line below have put it there for now
        trajectories = self.base_trajectory_generator()
        self.trajectory_deletion(occupancy_grid, trajectories)
        self.optimisation(trajectories, states)

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
            # Add transformation
            trajectories.append(added_trajectory)
        return trajectories
    def get_current_position(self):
        pass;

    # ===========================================================
    # TRAJECTORY DELETION
    def trajectory_deletion(self, occupancy_grid, trajectories=None):
        if trajectories is None:
            # get trajectories
            trajectories, _ = self.generate_trajectories(5)

        # delete trajectories intersecting boundary
        self.get_inbound_trajectories(occupancy_grid, trajectories)

    def get_inbound_trajectories(self, occupancy_grid, trajectories):
        # go through each trajectory
        for i in range(len(trajectories)):
            # go through each point of the trajectory
            for j in range(len(trajectories[i].poses)):
                # get the x and y index for this pose
                cpose = trajectories[i].poses[j]
                xlist, ylist = self.get_coordinate_list(cone_map)
                x = self.search_list(xlist,cpose.position.x)
                y = self.search_list(ylist,cpose.position.y)
                # check value in occupancy grid
                if occupancy_grid[x][y] == 0:
                    # point on boundary - invalid trajectory
                    trajectories.pop(i)
                    break

    def search_list(self, ls, val):
        # binary search
        # number of elements dropped in binary search
        elem_dropped = 0
        while True:
            # get middle element
            mid = (len(ls) // 2) - 1
            # check middle element
            if val == ls[mid]:
                # index found, if list has one element increase mid-index by 1
                if mid == -1: mid = 0
                break
            elif val > ls[mid]:
                # count number elements to be dropped
                elem_dropped += mid + 1
                # update list
                ls[:] = ls[(mid + 1):]
            else:
                # update list
                ls[:] = ls[:mid]

        return elem_dropped + mid

    def get_coordinate_list(self, cone_map):
        # get min and max x and y values
        x1, x2, y1, y2 = self.get_map_size(cone_map)

        # create list
        resolution = 0.1
        xlist = np.arange(x1,x2,resolution)
        ylist = np.arange(y1,y2,resolution)

        return xlist, ylist

    def get_map_size(self, cone_map:ConeMap):
        #Get maximum andd minimum x and y
        x1 = 0
        x2 = 0
        y1 = 0
        y2 = 0
        for cone in cone_map.cones:
            x = cone.pose.pose.position.x
            y = cone.pose.pose.position.y
            theta = cone.pose.pose.orientation.w
            covariance = cone.pose.covariance
            if x < x1:
                x1 = x
            elif x > x2:
                x2 = x
            if y < y1:
                y1 = y
            elif y > y2:
                y2 = y

        return x1, x2, y1, y2

    # ===========================================================
    # OPTIMISATION
    def optimisation(self, trajectories=None, states=None):
        if trajectories is None:
            # get trajectories
            trajectories, states = self.generate_trajectories(5)

        # get best trajectory state
        best_state = self.get_best_trajectory_state(trajectories, states)

        return best_state

    def get_best_trajectory_state(self, trajectories, trajectories_states):
        # create an array of trajectory lengths
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








