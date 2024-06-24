import numpy as np
from gymnasium import Env
from gymnasium.spaces import Discrete, Dict, Box
from gymnasium.vector.utils import create_empty_array

## Car Environment class
# This class defines the environment for the car
# The action space is defined as Discrete with 3 actions
# The observation space is defined as angles and speed
class CarEnv(Env):
    def __init__(self, rl_environment_node):
        # Define action and observation space
        self.action_space = Discrete(3)
        self.max_timesteps = 50000
        self.timesteps = 0
        self.num_angles = 30

        self.timesteps_without_reward = 0

        self.observation_space = Dict({
            'angles': Box(low=0, high=180, shape=(self.num_angles,), dtype=np.float32),  # Angles from 0 to 180 degrees
            'speed': Box(low=-1.0, high=100.0, shape=(1,), dtype=np.float32)  # Speed from -1.0 to 100.0
        })

        self.rl_environment_node = rl_environment_node  
    
    def step(self, action):
        # Execute one time step within the environment
        self.timesteps += 1
        if self.timesteps >= self.max_timesteps:
            print("Max timesteps reached")
            raw_observation = self.rl_environment_node.get_observation()
            obs = create_empty_array(self.observation_space, n=1, fn=np.zeros)
            obs['angles'] = raw_observation['angles'][:self.num_angles]
            obs['speed'] = raw_observation['speed']
            obs = encode_obs(obs)
            return obs, 0, True, False, {}
        
        reward = self.rl_environment_node.get_reward() * 10

        if reward == 0:
            self.timesteps_without_reward += 1
        else:
            self.timesteps_without_reward = 0

        if self.timesteps_without_reward >= 2000:
            print("Timesteps: ", self.timesteps)
            raw_observation = self.rl_environment_node.get_observation()
            obs = create_empty_array(self.observation_space, n=1, fn=np.zeros)
            obs['angles'] = raw_observation['angles'][:self.num_angles]
            obs['speed'] = raw_observation['speed']
            
            obs = encode_obs(obs)
            return obs, 0, True, True, {}

        raw_observation = self.rl_environment_node.get_observation()
        obs = create_empty_array(self.observation_space, n=1, fn=np.zeros)
        obs['angles'] = raw_observation['angles'][:self.num_angles]
        obs['speed'] = raw_observation['speed']
        
        speed = obs['speed']
        if speed < 0:
            reward += -1

        obs = encode_obs(obs)

        self.rl_environment_node.publish_desired_speed(action)
        
        return obs, reward, False, False, {}
        
    def reset(self):
        # Reset the state of the environment to an initial state
        self.rl_environment_node.reset_environment()
        self.timesteps = 0
        self.timesteps_without_reward = 0

        raw_observation = self.rl_environment_node.get_observation()
        obs = create_empty_array(self.observation_space, n=1, fn=np.zeros)
        obs['angles'] = raw_observation['angles'][:self.num_angles]
        obs['speed'] = raw_observation['speed']
        obs = encode_obs(obs)
        return obs, {}
    
def encode_obs(states) -> np.array:
    angles = states['angles']
    speed = states['speed']
    speed_array = np.array([speed])  # Convert speed to a single-element array
    array = np.concatenate([angles, speed_array], axis=0) # Concatenate along the first axis
    return array