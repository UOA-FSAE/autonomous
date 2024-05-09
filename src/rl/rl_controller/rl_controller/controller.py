import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Pose, PoseArray
import threading
import time
import numpy as np
from gymnasium import Env
from gymnasium.spaces import Discrete, Dict, Box

# Constants
CAR_POSITION = Pose()
SELECTED_TRAJECTORY = None
DESIRED_SPEED = 3.0
TRACK_POINT_REACHED = 0
TURNING_ANGLES = []

class RLEnvironmentNode(Node):
    def __init__(self):
        super().__init__('rl_controller')
        self.get_logger().info("RL Controller Node started")
        
        # Initialize car environment variables
        self.car_position = CAR_POSITION
        self.selected_trajectory = SELECTED_TRAJECTORY
        self.desired_speed = DESIRED_SPEED

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

        self.turning_angles = angle_list

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
        return {
            'angles': self.turning_angles,
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
        print("TIMESTEPS REACHED")

        self.delete_car_pub.publish(String(data="test"))

        time.sleep(1)
        self.create_car_pub.publish(String(data="test"))

        # Publishing -1.0 resets the goal point
        time.sleep(1)
        self.track_point_reached = 0
        self.publish_desired_speed(-1.0)

def main(args=None):
    rclpy.init(args=args)
    rl_environment_node = RLEnvironmentNode()
    
    # running Environment node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(rl_environment_node,))
    thread.start()

    env = CarEnv(rl_environment_node)

    while (1):
        time.sleep(0.1)
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)

        print("obs : ", obs)

        if done:
            obs = env.reset()

    rl_environment_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Car Environment class
class CarEnv(Env):
    def __init__(self, rl_environment_node):
        # Define action and observation space
        self.action_space = Discrete(3)
        self.max_timesteps = 1000
        self.timesteps = 0
        self.num_angles = 30

        self.observation_space = Dict({
            'angles': Box(low=0, high=180, shape=(self.num_angles,), dtype=np.float32),  # Angles from 0 to 180 degrees
            'speed': Box(low=-1.0, high=100.0, shape=(1,), dtype=np.float32)  # Speed from -1.0 to 100.0
        })

        self.rl_environment_node = rl_environment_node  
    
    def step(self, action):
        # Execute one time step within the environment
        self.timesteps += 1
        if self.timesteps >= self.max_timesteps:
            return [], 0, True, {}
        
        reward = self.rl_environment_node.get_reward()

        observation = self.rl_environment_node.get_observation()

        self.rl_environment_node.publish_desired_speed(action)
        
        # return observation, reward, done, info
        return np.array(self.rl_environment_node.get_observation()), reward, False, {}
        
    def reset(self):
        # Reset the state of the environment to an initial state
        self.rl_environment_node.reset_environment()
        self.timesteps = 0
        return np.array(self.rl_environment_node.get_observation())


""" 
In While Loop
        print("  ")
        print("Step: ")
        print("obs: ", obs)
        print("reward: ", reward)
        print("done: ", done)
        print("info: ", info)
        print("action: ", action)
        print("desired speed: ", rl_environment_node.desired_speed) """

""" if self.rl_environment_node.turning_angles != []:
            self.turning_angles = self.rl_environment_node.turning_angles
            self.rl_environment_node.turning_angles = []
            
            print("timestep: ", self.timesteps)
            print("Car Position x y: ", self.rl_environment_node.car_position.position.x, self.rl_environment_node.car_position.position.y)
            print("Track Point x y: ", self.rl_environment_node.selected_trajectory[5].position.x, self.rl_environment_node.selected_trajectory[5].position.y)

            print("Angle list count: ", len(self.turning_angles))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[0:4]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[4:8]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[8:12]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[12:16]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[16:20]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[20:24]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[24:28]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[28:32]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[32:36]]))
            print(", ".join([f"{angle:.3f}" for angle in self.turning_angles[36:40]])) """