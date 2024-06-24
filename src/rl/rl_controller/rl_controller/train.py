import gymnasium as gym
import math
import random
from collections import namedtuple, deque
from itertools import count
import wandb
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

# Constants
BATCH_SIZE = 128
TAU = 0.005
NUM_EPISODES = 10

Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))

sweep_config = {
    "method": "bayes",
    "metric": {"name": "mean_reward", "goal": "maximize"},
    "parameters": {
        "lr": {"min": 1e-5, "max": 1e-1},
        "epsilon": {"min": 0.05, "max": 0.9},
        "gamma": {"min": 0.1, "max": 0.99},
    },
}


class Model():
    def __init__(self, gym_env):
        self.env = gym_env

        # if GPU is to be used
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Get number of actions from gym action space
        n_actions = self.env.action_space.n
        state, info = self.env.reset()
        n_observations = len(state)


        self.policy_net = DQN(n_observations, n_actions).to(self.device)
        self.target_net = DQN(n_observations, n_actions).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.policy_net.parameters())
        self.memory = ReplayMemory(10000)

        self.lr = 0.0
        self.eps = 0.0
        self.gamma = 0.0
        
    
    def select_action(self, state):
        sample = random.random()
        eps_threshold = self.eps
        if sample > eps_threshold:
            with torch.no_grad():
                # t.max(1) will return the largest column value of each row.
                # second column on max result is index of where max element was
                # found, so we pick action with the larger expected reward.
                return self.policy_net(state).max(1).indices.view(1, 1)
        else:
            return torch.tensor([[self.env.action_space.sample()]], device=self.device, dtype=torch.long)
        
    def optimize_model(self):
            if len(self.memory) < BATCH_SIZE:
                return
            transitions = self.memory.sample(BATCH_SIZE)
            # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
            # detailed explanation). This converts batch-array of Transitions
            # to Transition of batch-arrays.
            batch = Transition(*zip(*transitions))

            # Compute a mask of non-final states and concatenate the batch elements
            # (a final state would've been the one after which simulation ended)
            non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                                batch.next_state)), device=self.device, dtype=torch.bool)
            non_final_next_states = torch.cat([s for s in batch.next_state
                                                        if s is not None])
            state_batch = torch.cat(batch.state)
            action_batch = torch.cat(batch.action)
            reward_batch = torch.cat(batch.reward)

            # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
            # columns of actions taken. These are the actions which would've been taken
            # for each batch state according to policy_net
            state_action_values = self.policy_net(state_batch).gather(1, action_batch)

            # Compute V(s_{t+1}) for all next states.
            # Expected values of actions for non_final_next_states are computed based
            # on the "older" target_net; selecting their best reward with max(1).values
            # This is merged based on the mask, such that we'll have either the expected
            # state value or 0 in case the state was final.
            next_state_values = torch.zeros(BATCH_SIZE, device=self.device)
            with torch.no_grad():
                next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1).values
            # Compute the expected Q values
            expected_state_action_values = (next_state_values * self.gamma) + reward_batch

            # Compute Huber loss
            criterion = nn.SmoothL1Loss()
            loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))

            # Optimize the model
            self.optimizer.zero_grad()
            loss.backward()
            # In-place gradient clipping
            torch.nn.utils.clip_grad_value_(self.policy_net.parameters(), 100)
            self.optimizer.step()

    def train(self):
            wandb.init()

            self.lr = wandb.config.lr
            self.eps = wandb.config.epsilon
            self.gamma = wandb.config.gamma
            

            self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.lr)

            mean_reward = 0

            for i_episode in range(NUM_EPISODES):
                # Initialize the environment and get its state
                cumulative_reward = 0
                timesteps = 0
                state, info = self.env.reset()
                state = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
                for t in count():
                    action = self.select_action(state)
                    observation, reward, terminated, truncated, _ = self.env.step(action.item())
                    timesteps += 1
                    cumulative_reward += reward
                    reward = torch.tensor([reward], device=self.device)
                    done = terminated or truncated

                    if terminated:
                        next_state = None
                    else:
                        next_state = torch.tensor(observation, dtype=torch.float32, device=self.device).unsqueeze(0)

                    # Store the transition in memory
                    self.memory.push(state, action, next_state, reward)

                    # Move to the next state
                    state = next_state

                    # Perform one step of the optimization (on the policy network)
                    self.optimize_model()

                    # Soft update of the target network's weights
                    # θ′ ← τ θ + (1 −τ )θ′
                    target_net_state_dict = self.target_net.state_dict()
                    policy_net_state_dict = self.policy_net.state_dict()
                    for key in policy_net_state_dict:
                        target_net_state_dict[key] = policy_net_state_dict[key]*TAU + target_net_state_dict[key]*(1-TAU)
                    self.target_net.load_state_dict(target_net_state_dict)

                    if done:
                        break

                print('Complete Episode: ', i_episode, 'of ', NUM_EPISODES)

                wandb.log(
                    {
                        "Time steps": timesteps,
                        "reward": cumulative_reward
                    }
                )

                mean_reward += cumulative_reward

            mean_reward /= NUM_EPISODES

            wandb.log(
                {
                    "mean_reward": mean_reward
                }
            )

            torch.save(self.policy_net.state_dict(), 'model.pth')

            return True
    
    def main(self):
        sweep_id = wandb.sweep(sweep_config, project="fsae-car-rl")

        wandb.agent(sweep_id, function=self.train, count=100)

        
class DQN(nn.Module):

    def __init__(self, n_observations, n_actions):
        super(DQN, self).__init__()
        self.layer1 = nn.Linear(n_observations, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, n_actions)

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        return self.layer3(x)

class ReplayMemory(object):

    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


