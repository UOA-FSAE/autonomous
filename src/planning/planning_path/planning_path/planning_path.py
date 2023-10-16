#!/usr/bin/python3
import numpy as np
import time

import rclpy
from rclpy.node import Node
from cone_mapping.srv import ConeMappingService
from geometry_msgs.msg import Pose, PoseArray
from moa_msgs.msg import Cone, ConeMap
from moa_msgs.msg import BoundaryStamped
from ackermann_msgs.msg import AckermannDrive
# from message_filters import ApproximateTimeSynchronizer



class path_planning(Node):
    def __init__(self):
        super().__init__("path_planning")

        self.ackermann_pub = self.create_publisher(AckermannDrive, "/cmd_vel", 5)
        self.boundl_sub = self.create_subscription(BoundaryStamped, "track/bound_l", self.get_left_boundary, 5)
        self.boundr_sub = self.create_subscription(BoundaryStamped, "track/bound_r", self.get_right_boundary, 5)
        self.cone_map_cli = self.create_client(ConeMappingService, 'conemapservice')

        self.timer = self.create_timer(5, self.publish_best_state)

    # BEST TRAJECTORY PUBLISHER
    def publish_best_state(self):
        '''
        Publishes the best trajectory state every n seconds
        '''
        self.get_logger().info("5 seconds up - generating trajectories")

        # request cone map - client
        temp_time = time.time()
        while not self.cone_map_cli.wait_for_service(timeout_sec=2):
            self.get_logger().info('cone map service not available, waiting again...')
            if time.time() - temp_time > 1:
                self.timer.reset()
                return
        # call async
        self.future = self.cone_map_cli.call_async(ConeMappingService.Request())
        while not self.future:
            self.timer.cancel()
            self.get_logger().info('waiting for future service response')
        self.timer.reset()
        cone_map = self.future.result
        # generate trajectories
        trajectories, states = self.trajectory_generator(cone_map)
        # delete trajectories
        self.trajectory_deletion(trajectories)
        # find best trajectory
        self.best_state = self.optimisation(trajectories, states)

        # publish msg
        args = {"steering_angle": float(self.best_state),
                "steering_angle_velocity": 0.0,
                "speed": 5.0,
                "acceleration": 0.0,
                "jerk": 0.0}
        msg = AckermannDrive(**args)
        self.ackermann_pub(msg)
        print(self.best_state)

    def get_left_boundary(self, msg: BoundaryStamped) -> None: self.leftbound = msg.coords
    
    def get_right_boundary(self, msg: BoundaryStamped) -> None: self.rightbound = msg.coords

    # ===========================================================
    # TRAJECTORY GENERATION
    def single_trajectory_generator(self, steering_angle, position_vector, rotation_matrix):
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357 is used for
        # bicycle steering
        L = 1;
        R = L / np.tan(steering_angle);
        t_range = np.arange(0, np.pi/2, 0.01);
        trajectory_output = PoseArray();
        for individual_t in t_range:
            pose_input = Pose();
            x_pre_trans = np.cos(individual_t) * R
            y_pre_trans = np.sin(individual_t) * R
            post_trans_point = self.apply_transformation(position_vector, rotation_matrix, x_pre_trans, y_pre_trans);
            pose_input.Point.x = post_trans_point[0]
            pose_input.Point.y = post_trans_point[1]
            trajectory_output.poses.append(pose_input)
        return trajectory_output

    def trajectory_generator(self, cone_map):
        candidate_steering_angle = np.deg2rad(np.arange(-10, 10, 0.5))
        trajectories = []
        position_and_orientation = self.get_position_of_cart(cone_map)
        position_vector, rotation_matrix = self.get_transformation_matrix(position_and_orientation)
        for steering_angle in candidate_steering_angle:
            added_trajectory = self.single_trajectory_generator(steering_angle, position_vector, rotation_matrix)
            # Add transformation
            trajectories.append(added_trajectory)
        return trajectories, candidate_steering_angle

    def get_position_of_cart(self, cone_map):
        localization_data = cone_map.cones[0]
        x = localization_data.pose.pose.position.x
        y = localization_data.pose.pose.position.y
        theta = localization_data.pose.pose.orientation.w
        return x, y, theta

    def get_transformation_matrix(self, position_and_orientation):
        theta = position_and_orientation[2]
        cart_x = position_and_orientation[0]
        cart_y = position_and_orientation[1]

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        position_vector = np.array([cart_x], [cart_y])

        return position_vector, rotation_matrix

    def apply_transformation(self, position_vector, rotation_matrix, point_x, point_y):
        point = np.array([point_x], [point_y])
        transformed_point = np.matmul(rotation_matrix, point) + position_vector
        return transformed_point

    # ===========================================================
    # TRAJECTORY DELETION
    def trajectory_deletion(self, trajectories=None) -> None:
        '''
        Caller for trajectory deletion if there are trajectories

        Inputs
            trajectories (1darray): a list of PoseArrays 
        '''
        if trajectories is None:
            return 
        self.get_logger().info("Trajectory Deletion Started")
        self.get_inbound_trajectories(trajectories) 
        
    def get_inbound_trajectories(self, trajectories):
        '''
        Deletes trajectories that are on track boundaries

        Inputs
            trajectories (1darray): a list of PoseArrays 
        '''
        for T in trajectories:
            for cpose in T.poses:
                x = cpose.position.x 
                y = cpose.position.y
                # compare points with boundaries
                onBound = self.compare_with_boundary(x,y)
                if onBound:  
                    trajectories.pop([i for i in range(len(trajectories)) if T==trajectories[i]][0])
                    break
    
    def compare_with_boundary(self, x, y):
        '''
        Compares a coordinate with the left and right boundary coordinates of the track map

        inputs
            x (float): x position of coordinate
            y (float): y position of coordinate
        return
            (boolean): True if point on either boundary
        
        * Assumes same number of points are given for left and right boundary
        '''
        for i in range(len(self.leftbound)):
            blx, bly = self.leftbound[i]
            brx, bry = self.rightbound[i]
            # on left boundary
            lxyt = abs(x-blx) <= 1e-3 and abs(y-bly) <= 1e-3
            # on right boundary
            rxyt = abs(x-brx) <= 1e-3 and abs(y-bry) <= 1e-3
            # check if near boundary
            if lxyt or rxyt:
                return True
        return False


    # def search_list(self, ls, val):
    #     ls = list(ls)
    #     val = np.round(val,4)
    #     # binary search - assumes a sorted search list
    #     # number of elements dropped in binary search
    #     elem_dropped = 0
    #     while True:
    #         # get middle element
    #         mid = int(np.ceil(len(ls) / 2)) - 1
    #         try:
    #             # check middle element
    #             if val == ls[mid]:
    #                 # index found, if list has one element increase mid-index by 1
    #                 if mid == -1: mid = 0
    #                 break
    #             elif val > ls[mid]:
    #                 # count number elements to be dropped
    #                 elem_dropped += mid + 1
    #                 # update list
    #                 ls[:] = ls[(mid + 1):]
    #             else:
    #                 # update list
    #                 ls[:] = ls[:mid]
    #         except IndexError:
    #             # value doesn't exist
    #             return None

    #     return elem_dropped + mid

    # ===========================================================
    # OPTIMISATION
    def optimisation(self, trajectories=None, states=None):
        if trajectories is None:
            return
        self.get_logger().info("Optimisation Started")
        best_state = self.get_best_trajectory_state(trajectories, states)

        return best_state

    def get_best_trajectory_state(self, trajectories, states):
        tlens = np.zeros(len(trajectories))
        for i in range(len(trajectories)):
            tlens[i] = self.get_trajectory_length(trajectories[i])
        states = self.get_trajectory_state(states,np.argmax(tlens))

        return states

    def get_trajectory_length(self, trajectory: PoseArray):
        poses = trajectory.poses
        p0 = np.array([poses[0].position.x, poses[0].position.y])
        p1 = np.array([poses[1].position.x, poses[1].position.y])
        intL = np.linalg.norm(p1-p0,ord=2)
        arc_length = (len(poses)-1) * intL

        return arc_length

    def get_trajectory_state(self, states, idx):
        return states[idx]

    # # FOR DEBUGGING
    # def generate_trajectories(self, n):
    #     # list of all trajectories and states (for now just steering angle in rads)
    #     all_traj = []
    #     all_states = -np.random.random(n) + np.random.random(n)
    #     # make n pose arrays
    #     x = y = z = 0.0
    #     for i in range(n):
    #         # contains poses for ith pose array
    #         temp = []
    #         # make m poses with random coordinates
    #         for j in range(3):
    #             pose = Pose()
    #             pose.position.x, pose.position.y, pose.position.z = x, y, z
    #             # pose_array.poses = pose
    #             temp.append(pose)
    #             # calculate new x, y, z sqrt((x2-x1)^2+(y2-y1)^2) = length with x2 unknown
    #             x = np.sqrt(0.1**2) + x
    #         pose_array = PoseArray()
    #         pose_array.poses = temp
    #         # append pose array to all trajectories
    #         all_traj.append(pose_array)

    #     return all_traj, all_states


def main():
    rclpy.init()

    NDE = path_planning()

    try:
        rclpy.spin(NDE)
    except Exception as e:
        print(f"node spin error: {e}")

    NDE.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()








