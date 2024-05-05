import rclpy
import random
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose

#Plotting and mathematic related
import math
import numpy as np
#import matplotlib.pyplot as plt 
import time

class Cone_Mapper(Node):

    def __init__(self):
        super().__init__('cone_mapper')
        print("started")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            ConeMap,
            'cone_detection',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Cone Map Node Started")

        # Create cone map publisher
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)

        #Static matrix size KF, need to change afterward
        self.number_of_cones = 0 #Used for second iteration only, later on would need to have this number be dynamic
        self.matrix_size = 3 + self.number_of_cones * 2
        self.default_cone_covariance = [99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #Measurement and measurement collection
        self.cone_map_array_measured = np.array([[],[]]) #Measured cone position that contains latest data only (for plot)
        self.Cone_map_measured = ConeMap() #Measured cone position in cone map that contains latest data only

        self.cone_map_array_measured_all = np.array([[],[]]) #Measured cone position that contains all data (for plot)
        self.Cone_map_measured_all = ConeMap() #Measured cone position in cone map that contains all data

        #Prediction and final output
        self.cone_map_array = np.array([[],[],[]]) #Predicted cone position (for plot)
        self.Cone_map = self.produce_cone_map_message(0.0, 0.0, 0.0, self.cone_map_array)
        #self.Cone_map = ConeMap(); #Predicted cone position in cone map
        self.cone_map_state = np.array([[0]]* self.matrix_size)
        #ing measured cones and adding new cones to the map
        #Kalman filter constants
        self.Q_constant = 0.00025 # Processing noise (need to be tuned)
        self.R_constant = 0.01 # Measurement noise
        self.Q_matrix = np.eye(self.matrix_size) * self.Q_constant
        self.R_matrix = np.eye(self.matrix_size) * self.R_constant
        self.Q_matrix[0][0] = 1e-9
        self.Q_matrix[1][1] = 1e-9
        self.Q_matrix[2][2] = 1e-9
        self.R_matrix[0][0] = 1e-9
        self.R_matrix[1][1] = 1e-9
        self.R_matrix[2][2] = 1e-9

        #Generate covariance matrix
        identity_3x3 = np.eye(3)
        expanded_matrix = np.zeros((self.matrix_size, self.matrix_size))
        expanded_matrix[:3, :3] = identity_3x3
        result_matrix = np.eye(self.matrix_size) * 99999 - expanded_matrix * 99999
        self.covariance = result_matrix

        #self.Kalman_gain = 1;
        self.counter = 0


    def listener_callback(self, msg):
        self.kalman_filter_update(msg)
        self.publisher.publish(self.Cone_map)

####SLAM fucntion below################################################################################################################################
    
    def kalman_filter_update(self, msg : ConeMap):
        #Get measurement
        cone_map_measurement_unsorted = self.cone_map_local_to_global(msg)

        self.produce_sorted_cone_map(cone_map_measurement_unsorted)

        #Perform first prediction (Step 1 on doc)
        first_prediction_state, existing_covariance, color_list = self.convert_cone_map_to_state(self.Cone_map)

        #Perform first prediction of covariance (Step 2 on doc)
        covariance_predicted = existing_covariance + self.Q_matrix
        
        #Perform Kalman gain calculation (Step 3 on doc)
        measured_state, measured_covariance, measured_color = self.convert_cone_map_to_state(self.Cone_map_measured) #Measured covariance is unused
        prefit_residual = measured_state - first_prediction_state
        prefit_covariance = covariance_predicted + self.R_matrix
        prefit_covariance_inversed = np.linalg.inv(prefit_covariance)
        Kalman_gain = np.matmul(covariance_predicted, prefit_covariance_inversed)

        #Perform opttimized prediction of state calculation (Step 4 on doc)
        optimized_prediction_state = first_prediction_state + np.matmul(Kalman_gain, prefit_residual)

        #Perform opttimized prediction of covariance calculation (Step 5 on doc)
        optimized_covariance =  np.matmul((np.identity(self.matrix_size) - Kalman_gain), covariance_predicted)
        postfit_residual = measured_state - optimized_prediction_state
        
        #Update the finalized cone map with the finalized state and covariance
        self.Cone_map = self.convert_state_to_cone_map(optimized_prediction_state, optimized_covariance, color_list)


    def cone_map_local_to_global(self, msg: ConeMap) -> ConeMap:
        """This function converts all cones in a cone map from the local frame to the global frame

        Args:
            msg (ConeMap): cone map that contains all the measured cones in local frame

        Returns:
            cone_map_measurement_unsorted (ConeMap): cone map that contains all the measured cones in global frame
        """
        # Convert cone map message into cart's (x, y and theta) and list of cones (cones' x,y and color)
        x, y, theta, list_of_cones = self.convert_cone_map_to_data(msg)

        # Use cart's (x, y and theta) to get the cart's position vector and rotation matrix
        position_vector, rotation_matrix = self.get_coordinate_conversion_matrices(x, y, theta)

        # Convert cone's coordinates from local frame to global frame
        global_cone_columns = self.local_to_global(position_vector, rotation_matrix, list_of_cones)
        
        # Produce a cone map that contains all the measured cones in global frame
        cone_map_measurement_unsorted = self.produce_cone_map_message(x, y, theta, global_cone_columns)

        return cone_map_measurement_unsorted
    

    def convert_cone_map_to_data(self, msg: ConeMap) -> tuple:
        """This function extracts data from the cone map message

        Args:
            msg (ConeMap): cone map message

        Returns:
            x (float): cart's x coordinate
            y (float): cart's y coordinate
            theta (float): cart's rotation angle
            list_of_cones (np.array): 3*n np.array contains each cone's  x coordinate, y coordinate and color
        """
        # Get the first cone (The first cone in the cone map contains the cart's pose data)
        cart = msg.cones[0]

        # Extract the cart's pose data
        x, y, theta, *other = self.extract_data_from_cone(cart)

        # Get all the cones in the cone map
        list_of_cones = msg.cones[1:]

        # Store cones' data
        list_of_local_cones_x = []
        list_of_local_cones_y = []
        list_of_local_cones_color = []

        # Extract data from all cone messages and store them in a 3*n np.array
        length = len(list_of_cones)
        for index in range(length):
            individual_x, individual_y, individual_theta, individual_covariance, individual_color, individual_cone_id = self.extract_data_from_cone(list_of_cones[index])
            list_of_local_cones_x.append(individual_x)
            list_of_local_cones_y.append(individual_y)
            list_of_local_cones_color.append(individual_color)

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y, list_of_local_cones_color])
                
        return x, y, theta, list_of_cones
    

    def extract_data_from_cone(self, cone_input: Cone) -> tuple:
        """This function extracts data from the cone message

        Args:
            cone_input (Cone): cone message

        Returns:
            x (float): cone's x coordinate
            y (float): cone's y coordinate
            theta (float): cone's rotation angle (only applicable when the cone is the cart)
            covariance (list[float]): cone's covariance vector
            color (int): cone's color
            cone_id (int): cone's id
        """
        x = cone_input.pose.pose.position.x
        y = cone_input.pose.pose.position.y
        theta = cone_input.pose.pose.orientation.w
        covariance = cone_input.pose.covariance
        color = cone_input.colour
        cone_id = cone_input.id
        return x, y, theta, covariance, color, cone_id
    

    def get_coordinate_conversion_matrices(self, x: float, y: float, theta: float) -> tuple:
        """This function gets cart's position vector and rotation  matrix

        Args:
            x (float): cart's x coordinate
            y (float): cart's y coordinate
            theta (float): cart's rotation angle

        Returns:
            position_vector (np.array): cart's global position vector
            rotation_matrix (np.array): rotation matrix for cone's local coordinates
        """
        position_vector = np.array([[x],[y],[0]])
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0],[math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

        return position_vector, rotation_matrix
    

    def local_to_global(self, position_vector: np.array, rotation_matrix: np.array, list_of_cones: np.array) -> np.array:
        """This function converts all cones data (x, y, color) from the local frame to the global frame

        Args:
            position_vector (np.array): cart's global position vector
            rotation_matrix (np.array): rotation matrix for cone's local coordinates
            list_of_cones (np.array): 3*n np.array contains each cone's local x coordinate, y coordinate and color

        Returns:
            list_of_cones_output (np.array): 3*n np.array contains each cone's global x coordinate, y coordinate and color
        """
        list_of_cones_unrotated = np.matmul(rotation_matrix, list_of_cones)
        list_of_cones_output = list_of_cones_unrotated + position_vector
        return list_of_cones_output


    def produce_cone_map_message(self, x: float, y: float, theta: float, list_of_cones: np.array) -> ConeMap:
        """This function packs cart's data and all cones' data into a cone map message

        Args:
            x (float): cart's x coordinate
            y (float): cart's y coordinate
            theta (float): cart's rotation angle
            list_of_cones (np.array): 3*n np.array contains each cone's  x coordinate, y coordinate and color

        Returns:
            output_map (ConeMap): cone map message
        """
        output_map = ConeMap()
        cart = self.pack_cone_message(x, y, theta, np.zeros(36), 0, 0)
        output_map.cones.append(cart)
        list_of_cones_x, list_of_cones_y, list_of_cones_color = list_of_cones
        length = len(list_of_cones_x)
        for index in range(length):
            cone = self.pack_cone_message(list_of_cones_x[index], list_of_cones_y[index], 0.0, self.default_cone_covariance, int(list_of_cones_color[index]), index + 1)    # initialise each measured cone's covariance_vector to self.default_cone_covariance
            output_map.cones.append(cone)
        return output_map
    

    def pack_cone_message(self, x: float, y: float, theta: float, covariance_vector: list[float], color: int, cone_id: int) -> Cone:
        """This function packs a cone's data into a cone message

        Args:
            x (float): cone's x coordinate
            y (float): cone's y coordinate
            theta (float): cart's rotation angle (not applicable)
            covariance_vector (list[float]): cone's covariance_vector
            color (int): cone's color
            cone_id (int): cone's id

        Returns:
            Cone: cone message
        """
        output_cone = Cone()
        position = Point()
        orientation = Quaternion()
        pose_with_covariance = PoseWithCovariance()
        pose = Pose()
        position.x = x
        position.y = y
        orientation.w = theta
        pose.position = position
        pose.orientation = orientation
        pose_with_covariance.pose = pose
        pose_with_covariance.covariance = covariance_vector
        output_cone.pose = pose_with_covariance
        output_cone.id = cone_id
        output_cone.colour = color
        return output_cone


    def produce_sorted_cone_map(self, cone_map_measurement_unsorted):
        #Sort cones that is measured into the cones that are logged into the map. If the cone is new, add new logged cone.
        self.Cone_map_measured = self.sort_and_add_cones(cone_map_measurement_unsorted)


    def sort_and_add_cones(self, cone_map_measurement_input : ConeMap) -> ConeMap:
        output = ConeMap()
        #Include cart localization info first
        output.cones.append(cone_map_measurement_input.cones[0])
        
        #Collect existing cones
        predicted_cones = self.Cone_map.cones[1:]
        #Collect upcoming measurement of the cones
        measured_cones = cone_map_measurement_input.cones[1:]
        
        #Sort existing cones
        matching_flag = False
        for cone in predicted_cones:
            #For each existing cone, check whether there is any measurement that is within the specified radius match_radius, and append the measurement if there is any and remove the measurement from measured_cones to avoid this measurement to be checked again
            match_radius = 0.5
            matching_flag = False
            predict_x, predict_y, predict_theta, predict_covaraince, predicted_color = self.extract_data_from_cone(cone)
            for measured_cone in measured_cones:
                measure_x, measure_y, measure_theta, measure_covariance, measured_color = self.extract_data_from_cone(measured_cone)
                if self.is_same_cone(predict_x, predict_y, measure_x, measure_y, match_radius) and predicted_color == measured_color:
                    output.cones.append(measured_cone)
                    measured_cones.remove(measured_cone)
                    matching_flag = True
                    break
            #If there is no measurement matches with the existing cone that is checking, the existing cone's reading will be appended to the output.
            if not(matching_flag):
                output.cones.append(cone)

        #If there are measurements that is not classified into the existing cones, the measurement will be added into the existing cones list as the new cones found.
        for left_cone in measured_cones:
            output.cones.append(left_cone)
            self.Cone_map.cones.append(left_cone)

        #Update Q and R matrix for change of number of cones.
        self.update_matrix()
        return output

    def update_matrix(self):
        self.number_of_cones = len(self.Cone_map.cones) - 1 #Used for second iteration only, later on would need to have this number be dynamic
        self.matrix_size = 3 + self.number_of_cones * 2
        self.Q_matrix = np.eye(self.matrix_size) * self.Q_constant
        self.R_matrix = np.eye(self.matrix_size) * self.R_constant
        self.Q_matrix[0][0] = 1e-9
        self.Q_matrix[1][1] = 1e-9
        self.Q_matrix[2][2] = 1e-9
        self.R_matrix[0][0] = 1e-9
        self.R_matrix[1][1] = 1e-9
        self.R_matrix[2][2] = 1e-9


    def convert_cone_map_to_state(self, cone_map : ConeMap) -> (np.array, np.array, np.array):
        list_of_cones = cone_map.cones[1::1]
        cart_info = cone_map.cones[0]
        number_of_cones = len(cone_map.cones) - 1

        covariance_size = number_of_cones * 2 + 3
        output_covariance = np.zeros((covariance_size, covariance_size))
        output_color = np.array([])
        
        x, y, theta, covariance_vector, color = self.extract_data_from_cone(cart_info)

        output_covariance[:3,:3] = self.convert_covariance_vector_to_matrix(covariance_vector, True)

        output_vector = np.array([[x], [y], [theta]])

        for index in range(0,len(list_of_cones),1):
            individual_x, individual_y, individual_theta, individual_covariance_vector, individual_color = self.extract_data_from_cone(list_of_cones[index])
            output_vector = np.append(output_vector, [[individual_x], [individual_y]], axis = 0)
            matrix_index = 3 + 2 * index
            output_covariance[matrix_index : matrix_index + 2, matrix_index : matrix_index + 2] = self.convert_covariance_vector_to_matrix(individual_covariance_vector, False)
            output_color = np.append(output_color, individual_color)
        return output_vector, output_covariance, output_color

    def convert_state_to_cone_map(self, state_vector : np.array, covariance : np.array, color_list : np.array) -> ConeMap:
        cart_info = state_vector[0:3:1]
        list_of_cones = state_vector[3::1]
        
        #Covariance matrix
        cart_covariance = covariance[:3, :3]
        cone_covariance = covariance[3:, 3:]

        cart_covariance_vector = self.convert_covariance_to_covariance_vector(cart_covariance)
        
        output_conemap = ConeMap()
        localization_cone = self.pack_cone_message(cart_info[0][0],cart_info[1][0],cart_info[2][0],0, cart_covariance_vector, 0)
        output_conemap.cones.append(localization_cone)

        index_for_color = 0
        for index in range(0, len(list_of_cones), 1):
            if index % 2 == 0:
                individual_cone_covariance = cone_covariance[index:index+2, index:index+2]
                individual_cone_covariance_vector = self.convert_covariance_to_covariance_vector(individual_cone_covariance)
                individual_cone = self.pack_cone_message(list_of_cones[index][0],list_of_cones[index + 1][0],0.0,index + 1,individual_cone_covariance_vector, int(color_list[index_for_color]))
                output_conemap.cones.append(individual_cone)
                index_for_color += 1

        #print(output_conemap);
        return output_conemap

    def convert_covariance_to_covariance_vector(self, covariance_matrix):
        #Input: covariance_matrix: n x n numpy array matrix
        #Output: float64[36] array
        matrix_size = len(covariance_matrix)
        full_covariance_matrix = np.zeros((6, 6))
        full_covariance_matrix[:matrix_size, :matrix_size] = covariance_matrix
        output_vector = []
        for row_vector in full_covariance_matrix:
            for element in row_vector:
                output_vector.append(element)
        return output_vector

    def convert_covariance_vector_to_matrix(self, covariance_vector, is_cart : bool) -> np.array:
        #Input: covariance_vector: float[36] array;
        #Output: covariance_matrix: either 3x3 or 2x2 (if is cart then 3x3 otherwise 2x2)
        full_covariance_matrix = np.zeros((6,6))
        index = 0
        for row_index in range(0, len(full_covariance_matrix), 1):
            for col_index in range(0, len(full_covariance_matrix), 1):
                full_covariance_matrix[row_index, col_index] = covariance_vector[index]
                index += 1

        if is_cart:
            return full_covariance_matrix[:3,:3]
        else:
            return full_covariance_matrix[:2,:2]


    def produce_unique_cone_list(self) -> np.array:
        cone_x_positions = self.cone_map_array_measured_all[0]
        cone_y_positions = self.cone_map_array_measured_all[1]
        number_of_cones = len(cone_x_positions)
        unique_cones_x = []
        unique_cones_y = []
        
        for index in range(0, number_of_cones, 1):
            if not(self.is_repeating(cone_x_positions[index], cone_y_positions[index], unique_cones_x, unique_cones_y, 0)):
                unique_cones_x.append(cone_x_positions[index])
                unique_cones_y.append(cone_y_positions[index])
        return np.array([unique_cones_x, unique_cones_y])


    def is_repeating(self, cone_x : float, cone_y : float, target_cone_x_list, target_cone_y_list, tolerance : float) -> bool:
        number_of_cones = len(target_cone_x_list)
        for index in range(0, number_of_cones, 1):
            if self.is_same_cone(target_cone_x_list[index], target_cone_y_list[index], cone_x, cone_y, tolerance):
                return True
        return False
            
    def is_same_cone(self, cone_1_x : float, cone_1_y : float, cone_2_x : float, cone_2_y : float, tolerance : float) -> bool:
        distance_differences_x = cone_1_x - cone_2_x
        distance_differences_y = cone_1_y - cone_2_y
        distance_apart = math.sqrt(distance_differences_x ** 2 + distance_differences_y ** 2)
        return abs(distance_apart) <= tolerance
    

def main(args=None):
    rclpy.init(args=args)
    cone_mapper = Cone_Mapper()
    rclpy.spin(cone_mapper)
    cone_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
