#!/usr/bin/python3

# imports
import numpy as np

import rclpy
from rclpy.node import Node
from moa_msgs.msg import Cone, ConeMap
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, Pose

import os
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, Sequence
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy

class ReinforcementLearningEnv(Node, Env):
    """reinforcement learning uses a policy to find out what action would be best for this state
    The action is defined as the highest probability of it happening given this state and parameters - I THINK SO"""
    def __init__ (self):
        super().__init__("RL")
        # ROS NODE CODE
        self.declare_parameters(
            namespace='',
            parameters=[
                ('savebounds', False)
            ]
        )
        # attributes
        self._saveBounds = self.get_parameter("savebounds").get_parameter_value().bool_value
        self._learn = True

        # subscribers
        self.create_subscription(ConeMap, "cone_map", self.callback, 10)
        # publishers
        self.best_steering_angle_pub = self.create_publisher(Float32, "moa/selected_steering_angle", 10)
        self.path =  self.create_publisher(PoseArray, "moa/chosen_actions", 10)

        # ENVIRONMENT NODE
        # distance to centerline
        self.action_space = Box(-45,45, shape=(1,))
        # observation?
        self.observation_space = Box(-999,999,shape=(4,))
        # steering angle 
        self.steering_angle = 0
        self.episode_length = 100

    def step(self, action):
        # convert to radians
        self.steering_angle = action[0] * np.pi / 180

        # decrease run time
        self.episode_length -= 1

        # calculate reward 
        distance = self.getDistanceToCenter()
        reward = -distance

        if self.episode_length <= 0:
            done = True
        else:
            done = False

        info = {}   

        # publish chosen angle
        self.best_steering_angle_pub.publish(Float32(data=float(self.steering_angle)))
        # self.get_logger().info(f'chosen action published {self.steering_angle}')

        return [self.center_point[0], self.center_point[1], distance, self.steering_angle], reward, done, info
    
    def render(self):
        pass

    def reset(self):
        self.steering_angle = 0
        # obs = Box(-999,999,shape=(1,))
        self.episode_length = 100

        return self.steering_angle
    
    # helper methods
    def getDistanceToCenter(self):
        # get new point
        new_point = self.getNewPoint()
        
        # return distance 
        return self.getDistance(new_point, self.center_point)
    
    def getNewPoint(self):
        trajectory_output = PoseArray()
        steering_angle = self.steering_angle
        L = 1
        R = L / np.tan(steering_angle)
        t_range = np.arange(0, np.pi, 0.01)
        for i, individual_t in enumerate(t_range):
            pose_input = Pose()
            # pre transformation coordinates
            x_pre_trans = np.cos(individual_t) * R - R
            y_pre_trans = abs(np.sin(individual_t) * R)
            # post transformation coordinates (relative to car)
            current_location = [[self.car_location.position.x],[self.car_location.position.y]]
            post_trans_point = self.applyTransformation(steering_angle, current_location, x_pre_trans, y_pre_trans)
            x = post_trans_point[0][0]
            y = post_trans_point[1][0]
            pose_input.position.x = x
            pose_input.position.y = y
            trajectory_output.poses.append(pose_input)

            if i == 2: break
        
        # publish msg
        # self.get_logger().info("path for action published")
        self.path.publish(trajectory_output)

        return np.array([x,y])
    
    def applyTransformation(self ,action, position_vector, x_pre, y_pre):
        point = np.array([[x_pre], [y_pre]])
        rotation_matrix = self.getRotationMatrix(action)
        # matrix multiplication for rotation then translate from car position
        transformed_point = np.matmul(rotation_matrix, point) + position_vector
        return transformed_point
    
    def getRotationMatrix(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    
    def getDistance(self, p1, p2):
        return np.sqrt( sum( ( p1-p2 ) ** 2 ) )


    # ROS NODE
    def callback(self, msg:ConeMap):
        cones = msg.cones
        # get car location
        self.car_location = cones[0].pose.pose

        # loop through each cone
        self._leftboundary = []
        self._rightboundary = []
        for i in range(len(cones)):
            if i != 0:
                x = cones[i].pose.pose.position.x
                y = cones[i].pose.pose.position.y
                # blue - left
                if cones[i].colour == 0:
                    self._leftboundary.append([x,y])
                elif cones[i].colour == 2:
                    self._rightboundary.append([x,y])
            else:
                self.car_pose = cones[i].pose.pose

        # get center point
        self.center_point = self.getCenterPoint()

        # save coordinates for debugging
        if self._saveBounds:
            with open('/home/tanish/Documents/autonomous/src/planning/path_planning/path_planning/bound_coods', 'w') as fh:
                    xl=[i[0] for i in self._leftboundary]
                    yl=[i[1] for i in self._leftboundary]
                    xr=[i[0] for i in self._rightboundary]
                    yr=[i[1] for i in self._rightboundary]
                    for P in xl:
                        fh.write("{} ".format(P))
                    fh.write("\n")
                    for P in yl:
                        fh.write("{} ".format(P))
                    fh.write("\n")
                    for P in xr:
                        fh.write("{} ".format(P))
                    fh.write("\n")
                    for P in yr:
                        fh.write("{} ".format(P))
                    fh.write("\n")
                    fh.close()
            self._saveBounds = False

        # REINFORCEMENT LEARNING 
        if self._learn:
            # testing if envirnoment works 
            episodes = 5
            for episode in range(1, episodes+1):
                self.reset()
                done = False
                score = 0

                while not done:
                    self.render()
                    action = self.action_space.sample()
                    obs, reward, done, info = self.step(action)
                    score += reward
                print(f"Episode: {episode} score: {score}")

            # self.env.close()

            log_path = os.path.join(os.getcwd(),'src/planning/path_planning/path_planning/Training','Logs')
            # configure model
            # env = DummyVecEnv([lambda: self])
            env = self
            model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path, learning_rate=0.5)

            # model training
            self.get_logger().info("TRAINING MODEL")
            model.learn(total_timesteps=20000)
            # save and load model
            save_path = os.path.join(os.getcwd(),'src/planning/path_planning/path_planning/Training','SavedModels','model_ppo')
            model.save(save_path)
            del model
            self.model = PPO.load(save_path, env)

            self._learn = False

        # model testing
        print(evaluate_policy(self.model, env, n_eval_episodes=10))

    # helper
    def getCenterPoint(self):
        left_point = np.array(self._leftboundary[0])
        right_point = np.array(self._rightboundary[0])

        return (left_point + right_point) / 2



def main(args=None):
    
    rclpy.init(args=args)

    node = ReinforcementLearningEnv()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
