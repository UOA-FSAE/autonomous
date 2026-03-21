import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32
from fsae_interfaces.msg import ConeMap
from fsae_interfaces.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose

import math
import numpy as np
import threading

import kdtree

class Item(object):
    def __init__(self, x, y, index):
        self.coords = (x, y)
        self.index = index

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.index)


class Cone_Mapper(Node):

    def __init__(self):
        super().__init__('cone_mapper')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create cone detection subscriber
        self.subscription = self.create_subscription(
            ConeMap,
            'cone_detection',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # Create cone map publisher
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)

        # Existing cone map
        self.Cone_map = None

        # KDTree
        self.tree = None

        # Current cone id
        self.current_cone_id = 1

################################################################################ (parameters to tune)

        # Initial error in the estimate
        self.default_cone_covariance = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Error in measurement
        self.error_in_measurement = 2.5

        # Cone match radius
        self.match_radius = 0.1

################################################################################ (measure duration for each cone map update)

        # Create update duration publisher
        self.duration_publisher = self.create_publisher(Float32, 'duration', 10)

        # Record the total update duration
        self.total_time = 0

        # Record the number of updates
        self.counter = 0

        # Lock for thread safety
        self.lock = threading.Lock()

        # Create a timer to calculate the average duration for each cone map update
        self.timer = self.create_timer(10.0, self.average_time_callback)

    def average_time_callback(self):
        """This function calculates and publishes the average cone map update duration every 10s to the /duration topic and write to a file
        """
        # Calculate the average update duration for the most recent 10s
        with self.lock:
            try:
                average_duration = self.total_time / self.counter
            except ZeroDivisionError:
                average_duration = 0.0
            self.total_time = 0
            self.counter = 0

        # Write the average update duration to a file
        with open("/home/fsae/Documents/pang/cone_mapping_data/ikf.txt", "a") as file:
            file.write(f"{average_duration}\n")

        # Publishes the average update duration to the /duration topic
        duration_msg = Float32()
        duration_msg.data = average_duration
        self.duration_publisher.publish(duration_msg)

################################################################################ (cone_mapping functions)

    def listener_callback(self, msg: ConeMap) -> None:
        """This function updates the existing cone map and publish it to the /cone_map topic

        Args:
            msg (ConeMap): input cone map from the /cone_detection topic
        """
        # Update the existing cone map with new measurements
        before = self.get_clock().now()
        self.update_existing_cone_map(msg)
        after = self.get_clock().now()

        # Calculate the update duration in micro seconds
        duration = (after - before).nanoseconds / 1000
        with self.lock:
            self.total_time += duration
            self.counter += 1

        # Publishes the existing cone map to the /cone_map topic
        self.publisher.publish(self.Cone_map)


    def update_existing_cone_map(self, msg: ConeMap) -> None:
        """This function updates the existing cone map using new measurements

        Args:
            msg (ConeMap): input cone map message
        """
        # Get global measurements
        x, y, theta, global_cone_columns = self.cone_map_local_to_global(msg)

        # Collect all cone's (x, y)
        list_of_cones_x, list_of_cones_y, list_of_cones_color = global_cone_columns
        measured_points = [(list_of_cones_x[i], list_of_cones_y[i]) for i in range(len(list_of_cones_x))]

        # Update the existing cone map
        if self.Cone_map == None:
            # If the existing cone map is empty, pack all cone measurements and update the existing cone map
            self.Cone_map = self.produce_cone_map_message(x, y, theta, global_cone_columns)
            self.tree = kdtree.create([Item(coord[0], coord[1], index + 1) for index, coord in enumerate(measured_points)])
            self.current_cone_id = len(measured_points) + 1
        else:
            # Update the cart position in the existing cone map 
            cart = self.Cone_map.cones[0]
            cart.pose.pose.position.x =  x
            cart.pose.pose.position.y =  y
            cart.pose.pose.orientation.w = theta

            # If the existing cone map is not empty, find the closest cone for each newly measured cone
            for i, coord in enumerate(measured_points):
                point, distance = self.tree.search_nn(coord)
                # If the newly measured cone is in the match radius, this cone already exists in the existing cone map, update its coordinates
                if distance <= self.match_radius:
                    index = point.data.index
                    cone = self.Cone_map.cones[index]
                    point.data.coords = self.kalman_filtering(cone, coord)
                else:
                    # If the newly measured cone is not in the match radius, this cone does not exist in the existing cone map, add it to the existing cone map
                    cone = self.pack_cone_message(coord[0], coord[1], 0.0, self.default_cone_covariance, list_of_cones_color[i], self.current_cone_id)
                    self.Cone_map.cones.append(cone)
                    self.tree.add(Item(coord[0], coord[1], self.current_cone_id))
                    self.current_cone_id += 1

    
    def kalman_filtering(self, cone: Cone, coord: tuple) -> tuple:
        """This function update the x and y coordinate of of the input cone using kalman filtering

        Args:
            cone (Cone): input cone message
            coord (tuple): new measurement of (x, y) of the input cone
        
        Returns:
        new_estimate_x (float): cart's new estimated x coordinate
        new_estimate_y (float): cart's new estimated y coordinate
        """
        # Get the previous error_in_estimate, estimated x and estimated y coordinate
        prev_error_in_estimate = cone.pose.covariance[0]
        prev_estimate_x = cone.pose.pose.position.x
        prev_estimate_y = cone.pose.pose.position.y

        # Calculate the kalman gain
        kalman_gain = prev_error_in_estimate / (prev_error_in_estimate + self.error_in_measurement)

        # Calculate the new error_in_estimate, estimated x and estimated y coordinate
        new_estimate_x = prev_estimate_x + kalman_gain * (coord[0] - prev_estimate_x)
        new_estimate_y = prev_estimate_y + kalman_gain * (coord[1] - prev_estimate_y)
        new_error_in_estimate = (1 - kalman_gain) * (prev_error_in_estimate)

        # update x and y coordinate and error_in_estimate
        cone.pose.pose.position.x = new_estimate_x
        cone.pose.pose.position.y = new_estimate_y
        cone.pose.covariance[0] = new_error_in_estimate

        return new_estimate_x, new_estimate_y


    def cone_map_local_to_global(self, msg: ConeMap) -> tuple:
        """This function converts all cones in a cone map from the local frame to the global frame

        Args:
            msg (ConeMap): cone map that contains all the measured cones in local frame

        Returns:
            x (float): cart's x coordinate
            y (float): cart's y coordinate
            theta (float): cart's rotation angle
            global_cone_columns (np.array): 3*n np.array contains each cone's global x coordinate, y coordinate and color
        """
        # Convert cone map message into cart's (x, y and theta) and list of cones (cones' x,y and color)
        x, y, theta, list_of_cones = self.convert_cone_map_to_data(msg)

        # Use cart's (x, y and theta) to get the cart's position vector and rotation matrix
        position_vector, rotation_matrix = self.get_coordinate_conversion_matrices(x, y, theta)

        # Convert cone's coordinates from local frame to global frame
        global_cone_columns = self.local_to_global(position_vector, rotation_matrix, list_of_cones)

        return x, y, theta, global_cone_columns
    

    def convert_cone_map_to_data(self, msg: ConeMap) -> tuple:
        """This function extracts data from the cone map message

        Args:
            msg (ConeMap): cone map message

        Returns:
            x (float): cart's x coordinate
            y (float): cart's y coordinate
            theta (float): cart's rotation angle
            list_of_cones (np.array): 3*n np.array contains each cone's local x coordinate, y coordinate and color
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
        """This function gets cart's gloabl position vector and local coordinate rotation matrix

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
        output_cone.colour = int(color)
        return output_cone
    

    def produce_cone_map_message(self, x: float, y: float, theta: float, list_of_cones: np.array) -> ConeMap:
        """This function packs cart's data and all cones' data into a cone map message

        Args:
            x (float): cart's x coordinate
            y (float): cart's y coordinate
            theta (float): cart's rotation angle
            list_of_cones (np.array): 3*n np.array contains each cone's x coordinate, y coordinate and color

        Returns:
            output_map (ConeMap): cone map message
        """
        output_map = ConeMap()
        cart = self.pack_cone_message(x, y, theta, np.zeros(36), 0, 0)
        output_map.cones.append(cart)
        list_of_cones_x, list_of_cones_y, list_of_cones_color = list_of_cones
        length = len(list_of_cones_x)
        for index in range(length):
            cone = self.pack_cone_message(list_of_cones_x[index], list_of_cones_y[index], 0.0, self.default_cone_covariance, int(list_of_cones_color[index]), index + 1)
            output_map.cones.append(cone)
        return output_map


def main(args=None):
    rclpy.init(args=args)
    cone_mapper = Cone_Mapper()
    rclpy.spin(cone_mapper)
    cone_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
