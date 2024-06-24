import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Pose, PoseArray
import threading
import time
import numpy as np
from .environment import CarEnv
from . import train

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


# Constants
CAR_POSITION = Pose()
SELECTED_TRAJECTORY = None
DESIRED_SPEED = 3.0
TRACK_POINT_REACHED = 0
TURNING_ANGLES = []
TURNING_ANGLES_COUNT = 30

## RL Environment Node
# This node is responsible for passing the parameters to the RL algorithm
# and receiving the action from the RL algorithm to control the car
# The parameters passed are the car position, selected trajectory, and desired speed
# The action received is the desired speed
class RLEnvironmentNode(Node):
    def __init__(self):
        super().__init__('rl_controller')
        self.get_logger().info("RL Controller Node started")
        
        # Initialize car environment variables
        self.car_position = CAR_POSITION
        self.selected_trajectory = SELECTED_TRAJECTORY
        self.desired_speed = DESIRED_SPEED
        self.reset_in_progress = False

        self.track_point_reached = TRACK_POINT_REACHED
        self.turning_angles = TURNING_ANGLES
        
        # Subscribe to track point reached
        self.track_point_reached_sub = self.create_subscription(
            Bool,
            'moa/track_point_reached',
            self.track_point_reached_callback,
            5)
        
        # Subscribe to car position
        self.car_pos_sub = self.create_subscription(
            Pose,
            'car_position',
            self.car_pos_callback,
            5)
        
        self.selected_trajectory_sub = self.create_subscription(
            PoseArray,
            'moa/selected_trajectory',
            self.selected_trajectory_callback,
            5)

        # Publisher for desired speed
        self.desired_speed_pub = self.create_publisher(Float64, 'desired_speed', 5)

        self.delete_car_pub = self.create_publisher(String, "/race_controller/delete", 5)

        self.create_car_pub = self.create_publisher(String, "/race_controller/create", 5)

# Update car position
    def car_pos_callback(self, msg):
        self.car_position = msg

# Update track point reached
    def track_point_reached_callback(self, msg):
        self.track_point_reached += 1

# Update selected trajectory
    def selected_trajectory_callback(self, msg: PoseArray):
        self.selected_trajectory = msg.poses

        angle_list = []
        angle_offset = 0

        # Calculating turning angles from car position to track points
        for i in range(5,len(msg.poses)):
            angle = np.arctan2(msg.poses[i].position.y - self.car_position.position.y, msg.poses[i].position.x
                                - self.car_position.position.x)

            angle = abs(np.degrees(angle))
            if angle_list == []:
                angle_offset = angle
            angle_list.append(abs(angle - angle_offset))

        self.turning_angles = np.asarray(angle_list)

# Publish desired speed
    def publish_desired_speed(self, action):
        if action == -1.0:
            self.desired_speed = -1.0
            msg = Float64()
            msg.data = self.desired_speed
            self.desired_speed_pub.publish(msg)
            self.desired_speed = 3.0
            return

        msg = Float64()
        self.desired_speed += 0.1 * (action - 1)
        msg.data = self.desired_speed
        self.desired_speed_pub.publish(msg)

# Get observation
    def get_observation(self):
        # Return the current observation
        # return first 30 angles and speed as list

        if len(self.turning_angles) < TURNING_ANGLES_COUNT:
            angles = np.zeros(TURNING_ANGLES_COUNT)
            for i in range(len(self.turning_angles)):
                angles[i] = self.turning_angles[i]
            return {
                'angles': angles[:TURNING_ANGLES_COUNT],
                'speed': self.desired_speed
            }
        
        return {
            'angles': self.turning_angles[:TURNING_ANGLES_COUNT],
            'speed': self.desired_speed
        }
        
# Get reward
    def get_reward(self):
        reward = self.track_point_reached
        self.track_point_reached = 0
        return reward

# Reset environment
    def reset_environment(self):
        # Reset the environment to the initial state
        if self.reset_in_progress:
            return
        
        self.reset_in_progress = True

        self.delete_car_pub.publish(String(data="test"))

        time.sleep(1)
        self.create_car_pub.publish(String(data="test"))

        # Publishing -1.0 resets the goal point
        time.sleep(1)
        self.track_point_reached = 0
        self.publish_desired_speed(-1.0)
        self.reset_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    rl_environment_node = RLEnvironmentNode()
    
    # running Environment node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(rl_environment_node,))
    thread.start()

    env = CarEnv(rl_environment_node)

    model = train.Model(env)

    finished = model.main()

    if finished:
        print("Training finished")
    # rl_environment_node.get_logger().info("Observation: " + str(obs))

    rl_environment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





