# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mapping_interfaces.msg import ConeMap
from mapping_interfaces.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance;
from geometry_msgs.msg import Pose;

#Plotting and mathematic related
import math
import numpy as np
import matplotlib.pyplot as plt 
import time

class Cone_Mapper(Node):

    def __init__(self):
        super().__init__('cone_mapper')
        print("started")
        self.subscription = self.create_subscription(
            ConeMap,
            'cone_detect',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cone_map = np.array([[],[]]);
        self.output_message = ConeMap(); #Message output that is ready to be logged.
        self.counter = 0

    def listener_callback(self, msg):
        #self.get_logger().info('Mapped result: "%s"' % msg.cones)
        print("Listened")
        x, y, theta, list_of_cones = self.convert_message_to_data(msg)
        position_vector, rotation_matrix, list_of_cones = self.convert_to_input_matrix(x, y, theta - np.pi/2, list_of_cones);
        new_cone_columns = self.create_cone_map(position_vector, rotation_matrix, list_of_cones)
        self.cone_map = np.concatenate((self.cone_map, new_cone_columns), axis=1)
        self.cone_map = self.produce_unique_cone_list()
        self.output_message = self.produce_cone_map_message(self.cone_map)
        
        #self.get_logger().info('Mapped result: "%s"' % self.cone_map)
        print(self.counter);
        self.counter += 1;

        if self.counter % 50 == 0:
            plt.scatter(self.cone_map[0], self.cone_map[1], marker="x")
            #plt.scatter([76.5979, 96.1476, 92.0906, 62.8596, 26.6301],[76.2157, 90.4749, 16.2427, 74.0812, 17.7761])
            plt.show();
            time.sleep(1)
            
    def convert_message_to_data(self, data):
        #Convert from Cone Map to data that is processed in the node
        list_of_cones = data.cones[1::1];
        cart_info = data.cones[0];
        
        x, y, theta = self.extract_data_from_cone(cart_info)

        list_of_local_cones_x = [];
        list_of_local_cones_y = [];
        
        for index in range(0,len(list_of_cones),1):
            individual_x, individual_y, individual_theta = self.extract_data_from_cone(list_of_cones[index])
            list_of_local_cones_x.append(individual_x);
            list_of_local_cones_y.append(individual_y);

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y])
                
        return x, y, theta, list_of_cones;

    def extract_data_from_cone(self, cone_input):
        x = cone_input.pose.pose.position.x;
        y = cone_input.pose.pose.position.y;
        theta = cone_input.pose.pose.orientation.w;
        return x, y, theta
        
    def convert_to_input_matrix(self, x: float, y: float, theta: float, list_of_cones):
        #Converting input message from cone detection into calculatable messages
        #t, x, y, theta, (distance(unit), theta(rad)) lists
        position_vector = np.array([[x],[y]]);
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]]) #Inverse DCM
        #No transformation for list of cones right now

        return position_vector, rotation_matrix, list_of_cones;

    def create_cone_map(self, position_vector, rotation_matrix, list_of_cones):
        list_of_cones_unrotated = np.matmul(rotation_matrix, list_of_cones)
        list_of_cones_output = list_of_cones_unrotated + position_vector;
        return list_of_cones_output;

    def produce_unique_cone_list(self):
        cone_x_positions = self.cone_map[0];
        cone_y_positions = self.cone_map[1];
        number_of_cones = len(cone_x_positions);
        unique_cones_x = [];
        unique_cones_y = [];
        
        for index in range(0, number_of_cones, 1):
            if not(self.is_repeating(cone_x_positions[index], cone_y_positions[index], unique_cones_x, unique_cones_y)):
                unique_cones_x.append(cone_x_positions[index])
                unique_cones_y.append(cone_y_positions[index])
        return np.array([unique_cones_x, unique_cones_y])

    def produce_cone_map_message(self, list_of_cones):
        output_map = ConeMap();
        list_of_cones_x = list_of_cones[0];
        list_of_cones_y = list_of_cones[1];
        length = len(list_of_cones_x);
        for index in range(length):
            cone_input = self.pack_cone_message(list_of_cones_x[index], list_of_cones_y[index], 0.0, index);
            output_map.cones.append(cone_input);
        return output_map;

    def pack_cone_message(self,x,y,theta,cone_id):
        output_cone = Cone();
        position = Point();
        orientation = Quaternion();
        pose_with_covariance = PoseWithCovariance();
        pose = Pose();
        position.x = x;
        position.y = y;
        orientation.w = theta;
        pose.position = position;
        pose.orientation = orientation;
        pose_with_covariance.pose = pose;
        output_cone.pose = pose_with_covariance;
        output_cone.id = cone_id
        return output_cone

    def is_repeating(self, cone_x, cone_y, target_cone_x_list, target_cone_y_list):
        number_of_cones = len(target_cone_x_list);
        for index in range(0, number_of_cones, 1):
            if self.is_same_cone(target_cone_x_list[index], target_cone_y_list[index], cone_x, cone_y, 0.5):
                return True;
        return False
            
    def is_same_cone(self, cone_1_x, cone_1_y, cone_2_x, cone_2_y, tolerance):
        distance_differences_x = cone_1_x - cone_2_x;
        distance_differences_y = cone_1_y - cone_2_y;
        distance_apart = math.sqrt(distance_differences_x ** 2 + distance_differences_y ** 2);
        return abs(distance_apart) <= tolerance
        

def main(args=None):
    rclpy.init(args=args)

    cone_mapper = Cone_Mapper()

    rclpy.spin(cone_mapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
