import rclpy
import random
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance;
from geometry_msgs.msg import Pose;

#Plotting and mathematic related
import math
import numpy as np
#import matplotlib.pyplot as plt 
import time

class Cone_Mapper(Node):

# Initializer
    def __init__(self):
        super().__init__('cone_mapper')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.subscription = self.create_subscription(ConeMap, 'cone_detection', self.listener_callback, qos_profile)
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)

        # Clustering related tuning parameter
        self.default_standard_deviation = 0.5 # Also minimal standard deviation
        self.default_variance = self.default_standard_deviation ** 2
        self.default_covariance = [self.default_variance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.default_variance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        self.new_cone_if_n_sigma_exceed_this = 3

        # Clustering method initialization
        self.most_updated_cone_map = ConeMap()
        self.number_of_measurements_for_cones = []
        self.car_position = Pose()
        self.cone_id = 1

        # Cone deletion tune
        self.count_above_this_are_safe = 10
        self.count_rate_above_this_are_safe = 2
        self.period_for_cone_deletion = 1

        # Cone deletion initialization
        self.cone_deletion_timer = self.create_timer(self.period_for_cone_deletion, self.cone_deletion_callback)
        self.number_of_measurements_increase_rate = []
        self.previous_number_of_measurements = []

        self.get_logger().info("Cone Map Initialization Completed")

    def listener_callback(self, msg):
        msg_in_local_coordinate = msg
        msg_in_global_coordinate = self.transform_raw_input_to_global_coordinate(msg_in_local_coordinate)
        self.most_updated_cone_map = self.clustering_update(msg_in_global_coordinate)
        self.publisher.publish(self.most_updated_cone_map)
        #self.get_logger().info("Cone Map Published")

# Conversion to Global Coordinate
## Main
    def transform_raw_input_to_global_coordinate(self, msg : ConeMap):
        """Extract measurement state from the Cone Map message subscription

        Args:
            msg: Input ConeMap message from Cone detection

        Returns:
            None

        Raises:
            None
        """
        # Convert Cone Map message into position (x and y), orientation (theta) and list of cones
        x, y, theta, list_of_cones_in_local_coordinates = self.convert_message_to_data(msg)
        # Use list of cones and states (x, y and theta) to get the position vector and rotation matrix
        position_vector, rotation_matrix_local_to_global = self.get_coordinate_conversion_matrices(x, y, theta)
        # Conversion from local reference frame to global reference frame
        list_of_cones_in_global_coordinates = self.get_list_of_cones_in_global_coordinate(position_vector, rotation_matrix_local_to_global, list_of_cones_in_local_coordinates)
        # Get unsorted Cone Map that contains all measured cone map at moment
        output_in_ConeMap_type = self.produce_cone_map_message(x, y, theta, list_of_cones_in_global_coordinates)

        return output_in_ConeMap_type

## Utility Functions for Conversion
    def get_coordinate_conversion_matrices(self, x: float, y: float, theta: float) -> (np.array, np.array, np.array):
        '''Convert state and list_of_cones input into position vector, rotation matrix (DCM) and the matrix of list of cones

        Args:
            x: x position specified in float
            y: y position specified in float
            theta: theta orientation specified in float
            list_of_cones: np.array (numpy array) for matrix of cone positions in 2 by n matrix (n is number of cones recorded in input))

        Returns:
            position_vector: np.array of position vector of cart
            rotation_matrix: np.array DCM matrix to convert reading from local frame into global frame
            list_of_cones: np.array 2 x n matrix of the list of cones measured in global frame

        Raises:
            None
        '''
        position_vector = np.array([[x], [y], [0]]);
        rotation_matrix_local_to_global = np.array(
            [[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])  # Inverse DCM

        return position_vector, rotation_matrix_local_to_global

    def get_list_of_cones_in_global_coordinate(self, position_vector : np.array, rotation_matrix : np.array, list_of_cones : np.array) -> np.array:
        list_of_cones_unrotated = np.matmul(rotation_matrix, list_of_cones)
        list_of_cones_output = list_of_cones_unrotated + position_vector
        return list_of_cones_output;

# Clustering Method
## Main
    def clustering_update(self, msg: ConeMap):
        output = ConeMap()
        cone_for_car_coordinate = msg.cones[0]
        output.cones.append(cone_for_car_coordinate)

        measured_cones_in_list_type = list(msg.cones[1:])
        recorded_cones = self.most_updated_cone_map.cones[1:]

        # Sort the measurement to the existing cones
        index = 0
        for individual_recorded_cone in recorded_cones:
            if len(measured_cones_in_list_type) > 0:
                closest_measurement, distance = self.sort_recorded_cone_and_find_closest_measurement(individual_recorded_cone, measured_cones_in_list_type)
                if self.measurement_is_not_new_cone(distance, individual_recorded_cone):
                    self.number_of_measurements_for_cones[index] += 1
                    updated_recorded_cone = self.update_mean_covariance_count(individual_recorded_cone, closest_measurement, index)
                    measured_cones_in_list_type.pop(0)
                else:
                    updated_recorded_cone = individual_recorded_cone
            else:
                updated_recorded_cone = individual_recorded_cone
            output.cones.append(updated_recorded_cone)
            index += 1

        # If measurement_cones_in_list_type are not empty, add rest of cones as new measurements
        for cone_measurement_that_is_new in measured_cones_in_list_type:
            x, y, theta, _, color, _ = self.extract_data_from_cone(cone_measurement_that_is_new)
            new_cone = self.pack_cone_message(x, y, theta, self.cone_id, self.default_covariance, color)
            self.number_of_measurements_for_cones.append(1)
            np.append(self.number_of_measurements_increase_rate, 1)
            self.previous_number_of_measurements.append(0)
            output.cones.append(new_cone)
            self.cone_id += 1

        return output

    def update_mean_covariance_count(self, recorded_cone, measured_cone, index):
        x_record, y_record, theta_record, covariance_record, color_record, cone_id = self.extract_data_from_cone(recorded_cone)
        x_measure, y_measure, theta_measure, covariance_measure, color_measure, _ = self.extract_data_from_cone(measured_cone)
        variance_record = covariance_record[0]
        number_of_meaasurements = self.number_of_measurements_for_cones[index]
        # Update mean
        x_new_rec = self.get_new_mean(x_record, x_measure, number_of_meaasurements)
        y_new_rec = self.get_new_mean(y_record, y_measure, number_of_meaasurements)
        # Update covariance
        distance = self.distance_between_two_cones(recorded_cone, measured_cone)
        new_variance = self.get_new_variance(variance_record, distance, number_of_meaasurements)
        new_covariance = [new_variance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, new_variance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        output_cone = self.pack_cone_message(x_new_rec, y_new_rec, theta_record, cone_id, new_covariance, color_record)
        return output_cone

    def get_new_mean(self, x_old_record, x_measurement, count):
        return x_old_record + (x_measurement - x_old_record) / count

    def get_new_variance(self, old_variance, distance, count):
        return (count - 2) * old_variance / (count - 1) + distance ** 2 / count

    def sort_recorded_cone_and_find_closest_measurement(self, recorded_cone: Cone, measured_cones_in_list_type):
        measured_cones_in_list_type.sort(key=lambda x: self.distance_between_two_cones(x, recorded_cone))
        closest_measured_cone_to_given_existing_cone = measured_cones_in_list_type[0]
        cone_output = closest_measured_cone_to_given_existing_cone
        distance_output = self.distance_between_two_cones(closest_measured_cone_to_given_existing_cone, recorded_cone)
        return cone_output, distance_output

    def measurement_is_not_new_cone(self, distance, recorded_cone: Cone):
        x, y, theta, covariance, color, _ = self.extract_data_from_cone(recorded_cone)
        standard_deviation = np.sqrt(covariance[0])
        criterion_standard_deviation = max([standard_deviation, self.default_standard_deviation])
        return distance <= self.new_cone_if_n_sigma_exceed_this * criterion_standard_deviation

## Utility Functions for Clustering
    def convert_message_to_data(self, data: ConeMap) -> (float, float, float, np.array):
        """Convert cone map message into useable data

        Args:
          data: Input Cone Map message from Cone detection

        Returns:
          Tuple consists of:
          x: float for x position
          y: float for y position
          theta: float for orientation
          list_of_cones: 2 x n numpy list (np.array) that contains the position of the cones.

        Raises:

        """
        # Convert from Cone Map to data that is processed in the node
        list_of_cones = data.cones[1::1];

        # Extract the first cone in cone map as the cart localization data
        cart_info = data.cones[0];

        # Convert cart info to readable datas for cart
        x, y, theta, covariance, color, _ = self.extract_data_from_cone(cart_info)

        list_of_local_cones_x = [];
        list_of_local_cones_y = [];
        list_of_local_cones_color = [];

        # Convert message of cones to 2 by n matrix where n is the number of cones in the measurement message
        for index in range(0, len(list_of_cones), 1):
            individual_x, individual_y, individual_theta, individual_covariance, individual_color, _ = self.extract_data_from_cone(
                list_of_cones[index])
            list_of_local_cones_x.append(individual_x);
            list_of_local_cones_y.append(individual_y);
            list_of_local_cones_color.append(individual_color);

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y, list_of_local_cones_color])

        return x, y, theta, list_of_cones

    def extract_data_from_cone(self, cone_input : Cone):
        # For later process: We need to use quaternion orientation
        x = cone_input.pose.pose.position.x
        y = cone_input.pose.pose.position.y
        theta = cone_input.pose.pose.orientation.w
        covariance = cone_input.pose.covariance
        color = cone_input.colour
        cone_id = cone_input.id
        return x, y, theta, covariance, color, cone_id

# Delete non-existing cones
    def cone_deletion_callback(self):
        # Count > 100 are cones, count rate increasing > 5 are cones
        self.update_increase_rate()
        initial_indexes = [i for i in range(0, len(self.number_of_measurements_for_cones), 1)]
        low_count_index = self.get_index_that_has_low_count(initial_indexes)
        false_cone_index = self.get_index_that_has_low_increase(low_count_index)
        self.delete_selected_cones(false_cone_index)
        self.previous_number_of_measurements = self.number_of_measurements_for_cones.copy()

    def delete_selected_cones(self, index_input):
        reversed_index_input = sorted(index_input, reverse=True)
        for index in reversed_index_input:
            self.most_updated_cone_map.cones.pop(index + 1)
            self.number_of_measurements_for_cones.pop(index)
            self.previous_number_of_measurements.pop(index)
            self.number_of_measurements_increase_rate.pop(index)

    def update_increase_rate(self):
        if len(self.number_of_measurements_for_cones) > len(self.previous_number_of_measurements):
            differences = len(self.number_of_measurements_for_cones) - len(self.previous_number_of_measurements)
            self.previous_number_of_measurements = self.previous_number_of_measurements + [0] * differences
        elif len(self.number_of_measurements_for_cones) < len(self.previous_number_of_measurements):
            differences = len(self.previous_number_of_measurements) - len(self.number_of_measurements_for_cones)
            self.previous_number_of_measurements = self.previous_number_of_measurements + [0] * differences

        self.number_of_measurements_increase_rate = []
        for i in range(len(self.number_of_measurements_for_cones)):
            differences = self.number_of_measurements_for_cones[i] - self.previous_number_of_measurements[i]
            self.number_of_measurements_increase_rate.append(differences / self.period_for_cone_deletion)

    def get_index_that_has_low_count(self, index_input):
        output_list = []
        list_to_study = [self.number_of_measurements_for_cones[i] for i in index_input]
        index = 0
        for individual_count in list_to_study:
            if individual_count < self.count_above_this_are_safe:
                output_list.append(index_input[index])
            index += 1
        return output_list

    def get_index_that_has_low_increase(self, index_input):
        output_list = []
        list_to_study = [self.number_of_measurements_increase_rate[i] for i in index_input]
        index = 0
        for individual_count_rate in list_to_study:
            if individual_count_rate < self.count_rate_above_this_are_safe:
                output_list.append(index_input[index])
            index += 1
        return output_list

# Debug only: Get all datas

# Other Utility Functions
    def produce_cone_map_message(self, x : float, y : float, theta : float, list_of_cones : np.array) -> ConeMap: #Produce message from array input
        output_map = ConeMap()
        cart_input = self.pack_cone_message(x, y, theta, 0, np.zeros(36), 0)
        output_map.cones.append(cart_input)
        list_of_cones_x = list_of_cones[0]
        list_of_cones_y = list_of_cones[1]
        list_of_cones_color = list_of_cones[2]
        length = len(list_of_cones_x)
        for index in range(length):
            cone_input = self.pack_cone_message(list_of_cones_x[index], list_of_cones_y[index], 0.0, index + 1, self.default_covariance, int(list_of_cones_color[index]));
            output_map.cones.append(cone_input)
        return output_map

    def pack_cone_message(self, x : float, y : float, theta : float, cone_id : int, covariance_vector, color : int) -> Cone:
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

    def distance_between_two_cones(self, cone_1, cone_2):
        x1, y1, _, _, _, _ = self.extract_data_from_cone(cone_1)
        x2, y2, _, _, _, _ = self.extract_data_from_cone(cone_2)
        delta_x = x1 - x2
        delta_y = y1 - y2
        return (delta_x ** 2) **  (1/2) + (delta_y ** 2)  ** (1/2)

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
