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

#Plotting and mathematic related
import math
import numpy as np
import matplotlib.pyplot as plt 
import time

class Cone_Mapper(Node):

    def __init__(self):
        super().__init__('cone_mapper')
        self.subscription = self.create_subscription(
            String,
            'cone_detect',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cone_map = np.array([[],[]]);
        self.counter = 0

    def listener_callback(self, msg):
        self.get_logger().info('Mapped result: "%s"' % msg.data)
        x, y, theta, list_of_cones = self.convert_message_to_data(str(msg.data))
        position_vector, rotation_matrix, list_of_cones = self.convert_to_input_matrix(x, y, theta - np.pi/2, list_of_cones);
        new_cone_columns = self.create_cone_map(position_vector, rotation_matrix, list_of_cones)
        self.cone_map = np.concatenate((self.cone_map, new_cone_columns), axis=1)
        self.cone_map = self.produce_unique_cone_list()
        self.get_logger().info('Mapped result: "%s"' % self.cone_map)
        self.counter += 1;

        if self.counter % 50 == 0:
            plt.scatter(self.cone_map[0], self.cone_map[1], marker="x")
            #plt.scatter([76.5979, 96.1476, 92.0906, 62.8596, 26.6301],[76.2157, 90.4749, 16.2427, 74.0812, 17.7761])
            plt.show();
            time.sleep(1)

    def convert_message_to_data(self, data: str):
        #Convert from message to plausible data, this function might change as we implement the message types
        #t, x, y, theta, (distance(unit), theta(rad)) lists
        #output:
        #x: float; y: float; theta: float; list_of_cones: 2 x n matrix
        #print("----xxxxx------")
        #print(data);
        list_of_messages = data.split(",");
        x = float(list_of_messages[1]);
        y = float(list_of_messages[2]);
        theta = float(list_of_messages[3]);

        list_of_cone_messages = list_of_messages[4::1];
        list_of_local_cones_x = [];
        list_of_local_cones_y = [];
        
        for index in range(0,len(list_of_cone_messages),1):
            if index % 2 == 0:
                individual_x = float(list_of_cone_messages[index].strip()[1::1]);
                individual_y = float(list_of_cone_messages[index + 1].strip()[0:len(list_of_cone_messages[index + 1].strip()) - 1:1]);
                list_of_local_cones_x.append(float(individual_x));
                list_of_local_cones_y.append(float(individual_y));

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y])
                
        return x, y, theta, list_of_cones;
        
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
