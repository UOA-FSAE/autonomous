#!/usr/bin/env python3
# Python imports
from typing import Optional
import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from moa_msgs.msg import ConeMap
from ackermann_msgs.msg import AckermannDrive



class head_to_goal_control_algorithm(Node):
    def __init__(self):
        super().__init__("Head_To_Goal_Controller")
        self.get_logger().info("Head to Goal Controller Node Started")

        # Constant to tune (touch me please it makes me feel horny ahhhhhhh!)
        ## Tuning for look ahead distance
        self.look_up_distance = 5
        self.cancel_distance = 2;
        ## Tuning for PID controller
        self.P = 10
        self.max_steering_angle = 20.0
        ## Current speed setting
        self.current_speed = 1

        # Initializer (normally don't touch)
        self.steering_angle = 0
        self.pos = (0,0)

        # subscribe to best trajectory
        self.best_trajectory_sub = self.create_subscription(PoseArray, "moa/selected_trajectory", self.selected_trajectory_handler, 5)
        self.cone_map_sub = self.create_subscription(ConeMap, "cone_map", self.main_hearback, 5)
        self.cmd_vel_pub = self.create_publisher(AckermannDrive, "/drive", 5)
        self.cmd_vis_pub = self.create_publisher(AckermannDrive, "/drive_vis", 5)
        self.track_point_pub = self.create_publisher(Pose, "moa/track_point", 5)

    def main_hearback(self, msg: ConeMap):
        # Update car's current location and update transformation matrix
        self.car_pose = msg.cones[0].pose.pose
        self.position_vector, self.rotation_matrix_l2g, self.rotation_matrix_g2l = self.convert_to_transformation_matrix(self.car_pose.position.x, self.car_pose.position.y, self.car_pose.orientation.w)

        # Before proceed, check whether we have a trajectory input
        if hasattr(self, "trajectory_in_global_frame"):
            # Update destination point to track
            self.update_track_point(self.trajectory_in_global_frame)

            # Get expected steering angle to publish
            self.steering_angle = self.get_steering_angle(self.Pose_to_track_in_global_frame)

            self.steering_angle = self.saturating_steering(self.steering_angle)
            # self.get_logger().info(f"Set steering angle to {self.steering_angle * self.P}")

        else:
            self.steering_angle = 0
            self.get_logger().info("Warning: no trajectory found, will set steering angle to 0!!!!")

        # Publish command for velocity
        self.publish_ackermann()

    def selected_trajectory_handler(self, msg: PoseArray):
        self.trajectory_in_global_frame = msg

    def saturating_steering(self, steering_angle):
        saturation = self.max_steering_angle
        if steering_angle > saturation:
            steering_angle = saturation
        elif steering_angle < -1 * saturation:
            steering_angle = -1 * saturation

        return steering_angle


    # Coordinate tranformer
    def convert_to_transformation_matrix(self, x: float, y: float, theta: float) -> (
            np.array, np.array, np.array):
        '''Convert state and list_of_cones input into position vector, rotation matrix (DCM) and the matrix of list of cones

        Args:
            x: x position specified in float
            y: y position specified in float
            theta: theta orientation specified in float
            list_of_cones: np.array (numpy array) for matrix of cone positions in 2 by n matrix (n is number of cones recorded in input))

        Returns:
            position_vector: np.array of position vector of cart
            rotation_matrix: np.array DCM matrix to convert reading from local frame into global frame
            list_of_cones: np.array 2 x n matrix of the list of cones measured in global frame

        Raises:
            None
        '''
        position_vector = np.array([[x], [y]])
        rotation_matrix_l2g = np.array(
            [[math.cos(theta), -math.sin(theta)],
             [math.sin(theta), math.cos(theta)]])  # local coordinate to global coordinate matrix
        rotation_matrix_g2l = np.array(
            [[math.cos(theta), math.sin(theta)],
             [-math.sin(theta), math.cos(theta)]])  # global coordinate to local coordinate matrix

        return position_vector, rotation_matrix_l2g, rotation_matrix_g2l


    def get_track_point_in_local_frame(self, track_point_in_global_frame: Pose):
        if (hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l")):
            track_point_in_local_frame = Pose()
            position_input = np.array(
                [[track_point_in_global_frame.position.x], [track_point_in_global_frame.position.y]])
            position_output = np.matmul(self.rotation_matrix_g2l, (position_input - self.position_vector))
            track_point_in_local_frame.position.x = float(position_output[0])
            track_point_in_local_frame.position.y = float(position_output[1])
            return track_point_in_local_frame
        else:
            self.get_logger.info("Warning: Local pose message not transformed to global frame")
            return track_point_in_global_frame


    def get_track_point_in_global_frame(self, track_point_in_local_frame: Pose):
        if (hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l")):
            track_point_in_global_frame = Pose()
            position_input = np.array(
                [[track_point_in_local_frame.position.x], [track_point_in_local_frame.position.y]])
            position_output = np.matmul(self.rotation_matrix_l2g, position_input) + self.position_vector
            track_point_in_global_frame.position.x = float(position_output[0])
            track_point_in_global_frame.position.y = float(position_output[1])
            return track_point_in_global_frame
        else:
            self.get_logger.info("Warning: Local pose message not transformed to global frame")
            return track_point_in_local_frame

# Picking and maintaining a track point
    def update_track_point(self, msg: PoseArray): #Main logic
        # Pick new tracking point if no tracking point is selected or old tracking point is no longer visible
        if self.need_new_track_point():
            self.Pose_to_track_in_global_frame = self.get_track_point_in_global_frame(msg)
        self.track_point_pub.publish(self.Pose_to_track_in_global_frame)

    def need_new_track_point(self):
        if not(hasattr(self, "Pose_to_track_in_global_frame")):
            return True
        else:
            Pose_to_track_in_local_frame = self.get_track_point_in_local_frame(self.Pose_to_track_in_global_frame)
            if self.track_point_still_visible(Pose_to_track_in_local_frame):
                if Pose_to_track_in_local_frame.position.y <= self.cancel_distance:
                    return True
                else:
                    return False
            else:
                return True

    def get_track_point_in_global_frame(self, trajectory_in_global_frame: PoseArray):
        if hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l"):
            sorted_trajectory_poses_in_global_frame = PoseArray()
            sorted_trajectory_poses_in_global_frame.poses = sorted(trajectory_in_global_frame.poses, key=lambda item: self.get_distance(self.get_track_point_in_local_frame(item)))
            for track_point_in_global_frame in sorted_trajectory_poses_in_global_frame.poses:
                track_point_in_local_frame = self.get_track_point_in_local_frame(track_point_in_global_frame)
                if self.track_point_distance_above_look_up_distance(track_point_in_local_frame):
                    return track_point_in_global_frame
            return self.car_pose

    def track_point_still_visible(self, track_point_in_local_frame: Pose):
        return track_point_in_local_frame.position.y >= 0

    def track_point_distance_above_look_up_distance(self, track_point_in_local_frame: Pose):
        return self.get_distance(track_point_in_local_frame) >= self.look_up_distance

    def get_distance(self, track_point_in_local_frame: Pose):
        x = track_point_in_local_frame.position.x
        y = track_point_in_local_frame.position.y
        return abs(((x ** 2) + (y ** 2)) ** (1/2))

### Calculate Steering Angle
    def get_steering_angle(self, Pose_to_track_in_global_frame: Pose):
        Pose_to_track_in_local_frame = self.get_track_point_in_local_frame(Pose_to_track_in_global_frame);
        error_in_x = Pose_to_track_in_local_frame.position.x
        return error_in_x * self.P

    def lateral_distance(self):
        return self.get_track_point_in_local_frame(self.Pose_to_track_in_global_frame).position.x

    def get_arc_radius(self, L, x):
        return L ** 2 / (2 * abs(x))

    def publish_ackermann(self):

        args1 = {"steering_angle": float(self.steering_angle),
                "steering_angle_velocity": 0.0,
                "speed": float(self.current_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg1 = AckermannDrive(**args1)

        print(msg1)

        args2 = {"steering_angle": float(self.steering_angle),
                "steering_angle_velocity": 0.0,
                "speed": float(self.current_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg2 = AckermannDrive(**args2)
        self.cmd_vel_pub.publish(msg1)
        self.cmd_vis_pub.publish(msg2)

def main(args=None):
    rclpy.init(args=args)

    pure_pursuiter = head_to_goal_control_algorithm()

    rclpy.spin(pure_pursuiter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pure_pursuiter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()