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

# Cone mapping node

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

        #Static matrix size KF, need to change afterward
        self.number_of_cones = 5; #Used for second iteration only, later on would need to have this number be dynamic
        self.matrix_size = 3 + self.number_of_cones * 2;
        self.default_cone_covariance = [99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        #Measurement and measurement collection
        self.cone_map_array_measured = np.array([[],[]]); #Measured cone position that contains latest data only (for plot)
        self.Cone_map_measured = ConeMap(); #Measured cone position in cone map that contains latest data only

        self.cone_map_array_measured_all = np.array([[],[]]); #Measured cone position that contains all data (for plot)
        self.Cone_map_measured_all = ConeMap(); #Measured cone position in cone map that contains all data

        #Prediction and final output
        self.cone_map_array = np.array([[0.0] * self.number_of_cones,[0.0] * self.number_of_cones]); #Predicted cone position (for plot)
        self.Cone_map = self.produce_cone_map_message(0.0, 0.0, 0.0, self.cone_map_array)
        #self.Cone_map = ConeMap(); #Predicted cone position in cone map
        self.cone_map_state = np.array([[0]]* self.matrix_size);
        
        #Kalman filter constants
        self.Q_constant = 0.00025; # Processing noise (need to be tuned)
        self.R_constant = 0.01; # Measurement noise
        self.Q_matrix = np.eye(self.matrix_size) * self.Q_constant;
        self.R_matrix = np.eye(self.matrix_size) * self.R_constant;

        #Generate covariance matrix
        identity_3x3 = np.eye(3)
        expanded_matrix = np.zeros((self.matrix_size, self.matrix_size))
        expanded_matrix[:3, :3] = identity_3x3
        result_matrix = np.eye(self.matrix_size) * 99999 - expanded_matrix * 99999
        self.covariance = result_matrix

        self.Kalman_gain = 1;
        self.counter = 0

    def listener_callback(self, msg):
        #self.get_logger().info('Mapped result: "%s"' % msg.cones)
        print("Listened")
        self.kalman_filter_update(msg)
        #self.get_measurement(msg) #For testing rev 0 function
        
        #self.get_logger().info('Mapped result: "%s"' % self.cone_map)
        #print(self.cone_map_array_measured);
        self.counter += 1;

        if self.counter % 50 == 0:
            plt.scatter(self.cone_map_array[0], self.cone_map_array[1], marker="x")
            #plt.scatter(self.cone_map_array_measured_all[0], self.cone_map_array_measured_all[1], marker="x")
            plt.scatter([76.5979, 96.1476, 92.0906, 62.8596, 26.6301],[76.2157, 90.4749, 16.2427, 74.0812, 17.7761])
            plt.show();
            time.sleep(1)

    def get_measurement(self, msg):
        x, y, theta, list_of_cones = self.convert_message_to_data(msg)
        position_vector, rotation_matrix, list_of_cones = self.convert_to_input_matrix(x, y, theta - np.pi/2, list_of_cones);
        new_cone_columns = self.create_cone_map(position_vector, rotation_matrix, list_of_cones)

        self.cone_map_array_measured = new_cone_columns;  #Produce latest measurement
        #print(self.cone_map_array_measured)
        self.Cone_map_measured = self.produce_cone_map_message(x, y, theta, self.cone_map_array_measured) #Produce map message
        #print(self.Cone_map_measured.cones)
        self.cone_map_array_measured_all = np.concatenate((self.cone_map_array_measured_all, new_cone_columns), axis=1)
        self.cone_map_array_measured_all = self.produce_unique_cone_list()
        self.Cone_map_measured_all = self.produce_cone_map_message(x, y, theta, self.cone_map_array_measured)

    def kalman_filter_update(self, msg):
        self.get_measurement(msg) #Get measurement

        #Perform first prediction
        first_prediction_state = self.cone_map_state;
        print("First state prediction\n", first_prediction_state)

        #Perform first prediction of covariance
        covariance_predicted = self.covariance + self.Q_matrix;
        #print("First covariance prediction", covariance_predicted);
        
        #Perform Kalman gain calculation
        measured_state, measured_covariance = self.convert_cone_map_to_state(self.Cone_map_measured) #Measured covariance is unused
        prefit_residual = measured_state - first_prediction_state;
        prefit_covariance = covariance_predicted + self.R_matrix;
        prefit_covariance_inversed = np.linalg.inv(prefit_covariance);
        self.Kalman_gain = np.matmul(covariance_predicted, prefit_covariance_inversed);
        #print("Kalman gain", self.Kalman_gain)

        #Perform opttimized prediction of state calculation
        #print("Measured state", self.convert_cone_map_to_state(self.Cone_map_measured))
        #print("Predicted state", first_prediction_state)
        optimized_prediction_state = first_prediction_state + np.matmul(self.Kalman_gain, prefit_residual);
        #print("Optimized prediction state", self.cone_map_state)

        #Perform opttimized prediction of covariance calculation
        optimized_covariance =  np.matmul((np.identity(self.matrix_size) - self.Kalman_gain), covariance_predicted);
        #optimized_covariance =  np.identity(self.matrix_size) - np.matmul(self.Kalman_gain, covariance_predicted);
        print("Optimized prediction covariance\n", self.covariance)
        postfit_residual = measured_state - optimized_prediction_state;
        
        self.covariance = optimized_covariance;
        self.cone_map_state = optimized_prediction_state;
        self.Cone_map = self.convert_state_to_cone_map(optimized_prediction_state, optimized_covariance);

        #Update array for plot
        x, y, theta, self.cone_map_array = self.convert_message_to_data(self.Cone_map);

    def convert_cone_map_to_state(self, cone_map):
        #Convert cone map into [x;y;theta;mx1;my1;mx2;my2;...] form
        list_of_cones = cone_map.cones[1::1];
        cart_info = cone_map.cones[0];
        number_of_cones = len(cone_map.cones) - 1;
        covariance_size = number_of_cones * 2 + 3;
        output_covariance = np.zeros((covariance_size, covariance_size));
        
        x, y, theta, covariance_vector = self.extract_data_from_cone(cart_info)

        output_covariance[:3,:3] = self.convert_covariance_vector_to_matrix(covariance_vector, True);

        output_vector = np.array([[x], [y], [theta]]);

        for index in range(0,len(list_of_cones),1):
            individual_x, individual_y, individual_theta, individual_covariance_vector = self.extract_data_from_cone(list_of_cones[index])
            output_vector = np.append(output_vector, [[individual_x], [individual_y]], axis = 0);
            matrix_index = 3 + 2 * index;
            output_covariance[matrix_index : matrix_index + 2, matrix_index : matrix_index + 2] = self.convert_covariance_vector_to_matrix(individual_covariance_vector, False)
        return output_vector, output_covariance

    def convert_state_to_cone_map(self, state_vector, covariance):
        cart_info = state_vector[0:3:1];
        list_of_cones = state_vector[3::1];
        
        #Covariance matrix
        cart_covariance = covariance[:3, :3];
        cone_covariance = covariance[3:, 3:];

        cart_covariance_vector = self.convert_covariance_to_covariance_vector(cart_covariance)
        
        output_conemap = ConeMap();
        #print("state",state_vector)
        localization_cone = self.pack_cone_message(cart_info[0][0],cart_info[1][0],cart_info[2][0],0, cart_covariance_vector);
        output_conemap.cones.append(localization_cone);
        #print("Covariance\n", covariance);
        #print("Cart covariance\n", cart_covariance);
        #print("Cone covariance\n", cone_covariance);
        #print("--")
        for index in range(0, len(list_of_cones), 1):
            if index % 2 == 0:
                individual_cone_covariance = cone_covariance[index:index+2, index:index+2];
                individual_cone_covariance_vector = self.convert_covariance_to_covariance_vector(individual_cone_covariance);
                individual_cone = self.pack_cone_message(list_of_cones[index][0],list_of_cones[index + 1][0],0.0,index + 1,individual_cone_covariance_vector);
            output_conemap.cones.append(individual_cone);

        return output_conemap;

    def convert_covariance_to_covariance_vector(self, covariance_matrix):
        #Input: covariance_matrix: n x n numpy array matrix
        #Output: float64[36] array
        matrix_size = len(covariance_matrix);
        full_covariance_matrix = np.zeros((6, 6));
        full_covariance_matrix[:matrix_size, :matrix_size] = covariance_matrix;
        output_vector = [];
        for row_vector in full_covariance_matrix:
            for element in row_vector:
                output_vector.append(element);
        return output_vector;

    def convert_covariance_vector_to_matrix(self, covariance_vector, is_cart):
        #Input: covariance_vector: float[36] array;
        #Output: covariance_matrix: either 3x3 or 2x2 (if is cart then 3x3 otherwise 2x2)
        full_covariance_matrix = np.zeros((6,6));
        index = 0;
        for row_index in range(0, len(full_covariance_matrix), 1):
            for col_index in range(0, len(full_covariance_matrix), 1):
                full_covariance_matrix[row_index, col_index] = covariance_vector[index];
                index += 1;

        if is_cart:
            return full_covariance_matrix[:3,:3];
        else:
            return full_covariance_matrix[:2,:2];
            
    def convert_message_to_data(self, data):
        #Convert from Cone Map to data that is processed in the node
        list_of_cones = data.cones[1::1];
        cart_info = data.cones[0];
        
        x, y, theta, covariance = self.extract_data_from_cone(cart_info)

        list_of_local_cones_x = [];
        list_of_local_cones_y = [];
        
        for index in range(0,len(list_of_cones),1):
            individual_x, individual_y, individual_theta, individual_covariance = self.extract_data_from_cone(list_of_cones[index])
            list_of_local_cones_x.append(individual_x);
            list_of_local_cones_y.append(individual_y);

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y])
                
        return x, y, theta, list_of_cones;

    def extract_data_from_cone(self, cone_input):
        x = cone_input.pose.pose.position.x;
        y = cone_input.pose.pose.position.y;
        theta = cone_input.pose.pose.orientation.w;
        covaraince = cone_input.pose.covariance;
        return x, y, theta, covaraince
        
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

    def pack_cone_message(self,x,y,theta,cone_id, covariance_vector):
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
        pose_with_covariance.covariance = covariance_vector;
        output_cone.pose = pose_with_covariance;
        output_cone.id = cone_id
        return output_cone

    def produce_unique_cone_list(self):
        cone_x_positions = self.cone_map_array_measured_all[0];
        cone_y_positions = self.cone_map_array_measured_all[1];
        number_of_cones = len(cone_x_positions);
        unique_cones_x = [];
        unique_cones_y = [];
        
        for index in range(0, number_of_cones, 1):
            if not(self.is_repeating(cone_x_positions[index], cone_y_positions[index], unique_cones_x, unique_cones_y)):
                unique_cones_x.append(cone_x_positions[index])
                unique_cones_y.append(cone_y_positions[index])
        return np.array([unique_cones_x, unique_cones_y])

    def produce_cone_map_message(self, x, y, theta, list_of_cones): #Produce message from array input
        output_map = ConeMap();
        cart_input = self.pack_cone_message(x, y, theta, 0, np.zeros(36));
        output_map.cones.append(cart_input);
        
        list_of_cones_x = list_of_cones[0];
        list_of_cones_y = list_of_cones[1];
        length = len(list_of_cones_x);
        for index in range(length):
            cone_input = self.pack_cone_message(list_of_cones_x[index], list_of_cones_y[index], 0.0, index + 1, self.default_cone_covariance);
            output_map.cones.append(cone_input);
        return output_map;

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
