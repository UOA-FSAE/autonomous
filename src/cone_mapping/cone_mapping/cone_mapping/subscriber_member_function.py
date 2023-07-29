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
import random
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
        self.number_of_cones = 0; #Used for second iteration only, later on would need to have this number be dynamic
        self.matrix_size = 3 + self.number_of_cones * 2;
        self.default_cone_covariance = [99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        #Measurement and measurement collection
        self.cone_map_array_measured = np.array([[],[]]); #Measured cone position that contains latest data only (for plot)
        self.Cone_map_measured = ConeMap(); #Measured cone position in cone map that contains latest data only

        self.cone_map_array_measured_all = np.array([[],[]]); #Measured cone position that contains all data (for plot)
        self.Cone_map_measured_all = ConeMap(); #Measured cone position in cone map that contains all data

        #Prediction and final output
        self.cone_map_array = np.array([[],[]]); #Predicted cone position (for plot)
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

        #self.Kalman_gain = 1;
        self.counter = 0

        #For testing purpose:
        self.real_x = [82.1083,73.7515,9.0773,31.9153,81.2393,8.3508,80.7517,83.43,12.1737,47.7871,7.479,83.9158,1.9508,11.3855,4.7073,59.0939,65.1652,90.6368,16.9003,68.6806,52.0282,59.4087,36.0385,1.7585,0.9228,83.9926,57.9914,75.4613,92.7,14.5266,95.7829,11.4884,20.9061,74.1405,93.9691,83.7772,78.7889,70.397,22.3445,28.8512,41.6785,64.9687,87.3374,12.886,90.214,61.7993,68.424,25.4803,77.7853,46.6661];
        self.real_y = [76.7874,82.8784,46.5864,58.0599,87.4731,4.0306,10.7985,58.7769,51.3741,94.3413,77.7735,85.6395,11.6419,89.5402,24.4232,13.6685,49.5905,10.8732,55.7474,74.138,68.6508,29.8633,50.6046,16.4749,69.5756,97.1017,55.2692,14.7197,86.3029,88.0514,50.5742,39.2341,98.8817,61.6964,5.6302,79.0565,98.6239,60.8165,66.09,92.8519,56.3865,2.8416,84.5369,65.8161,83.0636,0.5386,3.0325,42.7649,39.8977,29.6688];


    def listener_callback(self, msg):
        #self.get_logger().info('Mapped result: "%s"' % msg.cones)
        print("Listened")
        self.kalman_filter_update(msg)
        #self.get_measurement(msg) #For testing rev 0 function
        
        #self.get_logger().info('Mapped result: "%s"' % self.cone_map)
        #print(self.cone_map_array_measured);
        self.counter += 1;

        #if self.counter % 50 == 0:
            #plt.scatter(self.cone_map_array[0], self.cone_map_array[1], marker="x") #x marker for cone mapping
            #plt.scatter(self.cone_map_array_measured_all[0], self.cone_map_array_measured_all[1], marker=".") #. marker for all measurements taken
            #plt.scatter(self.real_x,self.real_y, marker = ".") #. marker for true position
            #plt.show();
            #time.sleep(1)
##############################
    def sort_and_add_cones(self, cone_map_measurement_input):
        output = ConeMap();
        output.cones.append(cone_map_measurement_input.cones[0]); #Include cart info first
        
        predicted_cones = self.Cone_map.cones[1:];  #Choose all cone object except for first one which is measurement of cart
        measured_cones = cone_map_measurement_input.cones[1:];
        
        #Sort existing cones
        matching_flag = False;
        for cone in predicted_cones:
            matching_flag = False;
            predict_x, predict_y, predict_theta, predict_covaraince = self.extract_data_from_cone(cone)
            for measured_cone in measured_cones:
                measure_x, measure_y, measure_theta, measure_covariance = self.extract_data_from_cone(measured_cone)
                if self.is_same_cone(predict_x, predict_y, measure_x, measure_y, 10):
                    output.cones.append(measured_cone);
                    measured_cones.remove(measured_cone);
                    matching_flag = True;
                    break;
            if not(matching_flag):
                output.cones.append(cone);
        
        print("predicted", self.convert_message_to_data(self.Cone_map)[3])
        print("measured", self.convert_message_to_data(cone_map_measurement_input)[3])
        print("output", self.convert_message_to_data(output)[3])

        #Add new cones that is not appeared
        for left_cone in measured_cones:
            output.cones.append(left_cone);
            self.Cone_map.cones.append(left_cone);

        #Update Q and R matrix
        self.update_matrix()
            
        return output;

    def update_matrix(self):
        self.number_of_cones = len(self.Cone_map.cones) - 1; #Used for second iteration only, later on would need to have this number be dynamic
        self.matrix_size = 3 + self.number_of_cones * 2;
        self.Q_matrix = np.eye(self.matrix_size) * self.Q_constant;
        self.R_matrix = np.eye(self.matrix_size) * self.R_constant;

    def get_measurement(self, msg):
        """Extract measurement state from the Cone Map message subscription

        Args:
          msg: Input Cone Map message from Cone detection

        Raises:
          
        """
        #Convert Cone Map message into position (x and y), orientation (theta) and list of cones
        x, y, theta, list_of_cones = self.convert_message_to_data(msg)
        #Use list of cones and states (x, y and theta) to get the position vector and rotation matrix
        position_vector, rotation_matrix, list_of_cones = self.convert_to_input_matrix(x, y, theta - np.pi/2, list_of_cones);
        #Conversion from local reference frame to global reference frame
        new_cone_columns = self.create_cone_map(position_vector, rotation_matrix, list_of_cones)
        self.cone_map_array_measured = new_cone_columns;  #Produce latest measurement
        
        #Get unsorted Cone Map that contains all measured cone map at moment
        cone_map_measurement_unsorted = self.produce_cone_map_message(x, y, theta, self.cone_map_array_measured) #Produce map message
        
        #Sort cones that is measured into the cones that are logged into the map. If the cone is new, add new logged cone.
        self.Cone_map_measured = self.sort_and_add_cones(cone_map_measurement_unsorted);

        #For Debug: Add the measured cone to the collection of all measurements (using this for debug purpose only)
        self.cone_map_array_measured_all = np.concatenate((self.cone_map_array_measured_all, new_cone_columns), axis=1)
        self.cone_map_array_measured_all = self.produce_unique_cone_list()
        self.Cone_map_measured_all = self.produce_cone_map_message(x, y, theta, self.cone_map_array_measured)

    def kalman_filter_update(self, msg):
        print("Before", len(self.Cone_map.cones))
        self.get_measurement(msg) #Get measurement
        print("After", len(self.Cone_map.cones))
        #Perform first prediction
        #first_prediction_state = self.cone_map_state;
        first_prediction_state, existing_covariance = self.convert_cone_map_to_state(self.Cone_map);
        #print("checkpoint\n",existing_covariance)
        #Perform first prediction of covariance
        #covariance_predicted = self.covariance + self.Q_matrix;
        covariance_predicted = existing_covariance + self.Q_matrix;
        
        #Perform Kalman gain calculation
        measured_state, measured_covariance = self.convert_cone_map_to_state(self.Cone_map_measured) #Measured covariance is unused
        prefit_residual = measured_state - first_prediction_state;
        prefit_covariance = covariance_predicted + self.R_matrix;
        prefit_covariance_inversed = np.linalg.inv(prefit_covariance);
        Kalman_gain = np.matmul(covariance_predicted, prefit_covariance_inversed);

        #Perform opttimized prediction of state calculation
        optimized_prediction_state = first_prediction_state + np.matmul(Kalman_gain, prefit_residual);

        #Perform opttimized prediction of covariance calculation
        optimized_covariance =  np.matmul((np.identity(self.matrix_size) - Kalman_gain), covariance_predicted);
        postfit_residual = measured_state - optimized_prediction_state;
        
        #self.covariance = optimized_covariance;
        #self.cone_map_state = optimized_prediction_state;
        self.Cone_map = self.convert_state_to_cone_map(optimized_prediction_state, optimized_covariance);
        #Update array for plot
        x, y, theta, self.cone_map_array = self.convert_message_to_data(self.Cone_map);

    def convert_cone_map_to_state(self, cone_map):
        #Convert cone map into [x;y;theta;mx1;my1;mx2;my2;...] form
        list_of_cones = cone_map.cones[1::1];
        cart_info = cone_map.cones[0];
        number_of_cones = len(cone_map.cones) - 1;
        #print(number_of_cones)
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
        '''
        
        '''
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
        """Convert cone map message into useable data

        Args:
          data: Input Cone Map message from Cone detection

        Returns:
          Tuple consists of:
          x: float for x position
          y: float for y position
          theta: float for orientation
          list_of_cones: 2 x n numpy list that contains the position of the cones.

        Raises:
          
        """
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
            if not(self.is_repeating(cone_x_positions[index], cone_y_positions[index], unique_cones_x, unique_cones_y, 0)):
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

    def is_repeating(self, cone_x, cone_y, target_cone_x_list, target_cone_y_list, tolerance):
        number_of_cones = len(target_cone_x_list);
        for index in range(0, number_of_cones, 1):
            if self.is_same_cone(target_cone_x_list[index], target_cone_y_list[index], cone_x, cone_y, tolerance):
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
