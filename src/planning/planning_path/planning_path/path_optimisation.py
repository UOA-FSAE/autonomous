#!/usr/bin/python3
import numpy as np
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from moa_msgs.msg import Cone, ConeMap, BoundaryStamped, AllTrajectories
from ackermann_msgs.msg import AckermannDrive


class path_planning(Node):
    def __init__(self):
        super().__init__("Path Planning & Trajectory Optimisation")

        # all trajectories publisher 
        self.all_traj_pub = self.create_publisher(AllTrajectories,"moa/trajectories",5)
        # subscribe to car states and cone map
        self.ackerman_sub = self.create_subscription(AckermannDrive, "/cmd_vel", self.get_current_states, 5)
        self.cone_map_sub = self.create_subscription(ConeMap,"cone_map",self.get_cone_map,5)

        # publish best trajectorys
        self.best_traj_pub = self.create_publisher(AckermannDrive, "moa/selected_trajectory", 5)
        # subscribe to all trajectories
        self.all_traj_sub = self.create_subscription(AllTrajectories, "moa/trajectories", self.get_all_trajectories, 5)
        # subscribe to cone map and track boundaries
        self.boundl_sub = self.create_subscription(BoundaryStamped, "track/bound_l", self.get_left_boundary, 5)
        self.boundr_sub = self.create_subscription(BoundaryStamped, "track/bound_r", self.get_right_boundary, 5)
        self.cone_map_sub = self.create_subscription(ConeMap, "cone_map", self.get_cone_map, 5)

        self.timer = self.create_timer(5, self.publish_trajectories)

    # BEST TRAJECTORY PUBLISHER
    def publish_trajectories(self):
        '''
        Publishes all generated trajectories once created every n seconds
        '''
        self.get_logger().info("5 seconds up - generating trajectories")

        # generate trajectories
        self.trajectories, states = self.trajectory_generator(self.cone_map)
        
        # publish to moa/trajectories
        trajectory_list = []
        for i in range(len(states)):
            args = {"steering_angle": float(states[i]),
                    "steering_angle_velocity": 0.0,
                    "speed": self.current_speed,
                    "acceleration": 0.0,
                    "jerk": 0.0}
            trajectory_list.append(AckermannDrive(**args))
        
        msg = AllTrajectories(trajectories = trajectory_list)
        self.all_traj_pub.publish(msg)

    def get_current_states(self, msg:AckermannDrive) -> None: 
        self.current_speed = msg.speed
        self.current_angle = msg.steering_angle
    
    def get_cone_map(self, msg:ConeMap) -> None: self.cone_map = msg.cones

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

    # CLASS TRAJECTORY_OPTIMISATION is combined as trajectory points are needed
    def get_all_trajectories(self, msg: AllTrajectories) -> None: 
        states = []
        trajs = msg.trajectories
        for i in range(len(trajs)):
            states.append(trajs[i].steering_angle)
        
        # trajectory deletion
        self.trajectory_deletion(self.trajectories, states)
        # optimisation
        best_state = self.optimisation(self.trajectories,states)

        # publish to moa/selected_trajectory
        args = {"steering_angle": float(best_state),
                "steering_angle_velocity": 0.0,
                "speed": self.current_speed,
                "acceleration": 0.0,
                "jerk": 0.0}
        msg = AckermannDrive(**args)
        self.best_traj_pub.publish(msg)
    
    def get_left_boundary(self, msg: BoundaryStamped) -> None: self.leftbound = msg.coords
    
    def get_right_boundary(self, msg: BoundaryStamped) -> None: self.rightbound = msg.coords

    def get_cone_map(self, msg:ConeMap) -> None: self.cone_map = msg.cones

    # ===========================================================
    # TRAJECTORY DELETION
    def trajectory_deletion(self, trajectories=None, states=None) -> None:
        '''
        Caller for trajectory deletion if there are trajectories

        Inputs
            trajectories (1darray): a list of PoseArrays 
        '''
        if trajectories is None:
            return 
        self.get_logger().info("Trajectory Deletion Started")
        self.get_inbound_trajectories(trajectories,states) 
        
    def get_inbound_trajectories(self, trajectories, states):
        '''
        Deletes trajectories that are on track boundaries

        Inputs
            trajectories (1darray): a list of PoseArrays 
        '''
        rm_inds = []
        for i, T in enumerate(trajectories):
            for cpose in T.poses:
                x = cpose.position.x 
                y = cpose.position.y
                # compare points with boundaries
                onBound = self.compare_with_boundary(x,y)
                if onBound:  
                    rm_inds.append(i)
                    # trajectories.pop([i for i in range(len(trajectories)) if T==trajectories[i]][0])
                    break
        [(trajectories.pop(index), states.pop(index)) for index in list(reversed(rm_inds))]
    
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

    def DEBUG_generate_trajectories(self, n):
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
                pose.position.x, pose.position.y = x, y
                # pose_array.poses = pose
                temp.append(pose)
                # calculate new x, y, z sqrt((x2-x1)^2+(y2-y1)^2) = length with x2 unknown
                x = np.sqrt(0.1**2) + x
            pose_array = PoseArray()
            pose_array.poses = temp
            # append pose array to all trajectories
            all_traj.append(pose_array)

        return all_traj, all_states


class trajectory_following(Node):
    def __init__(self):
        super().__init__("Trajectory Following")

        # publish p-controlled trajectory
        self.p_controlled_pub = self.create_publisher(AckermannDrive, "moa/drive", 5)
        # subscribe to best trajectory
        self.best_traj_sub = self.create_subscription(AckermannDrive, "moa/selected_trajectory", self.get_best_state, 5)
        self.current_states_sub = self.create_subscription(AckermannDrive, "/cmd_vel", self.get_current_states, 5)

    def get_current_states(self, msg: AckermannDrive) -> None: 
        self.current_speed = msg.speed
        self.current_angle = msg.steering_angle

    def get_control_error(self, csa, dsa): return csa-dsa

    def get_best_state(self, msg: AckermannDrive):
        error = self.get_control_error(self.current_angle, msg.steering_angle)
        # constant gain multiplier
        p_gain = 0.5
        # new steering angle output
        chosen_state = self.current_angle + error * p_gain

        # publish msg to /moa/drive
        args = {"steering_angle": float(chosen_state),
                "steering_angle_velocity": 0.0,
                "speed": self.current_speed,
                "acceleration": 0.0,
                "jerk": 0.0}
        msg = AckermannDrive(**args)
        self.p_controlled_pub(msg)
        self.get_logger().info(f"P-controlled trajectory steering angle published = {chosen_state}")
        print(chosen_state)


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








