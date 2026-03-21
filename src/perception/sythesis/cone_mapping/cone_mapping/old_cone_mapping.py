import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from fsae_interfaces.msg import ConeMap, Cones, Cone
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance

import math
import numpy as np

import kdtree

class Item(object):
    def __init__(self, x, y, data):
        self.coords = (x, y)
        self.data = data

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {}, {})'.format(self.coords[0], self.coords[1], self.data)


class Cone_Mapper(Node):
    def __init__(self):
        super().__init__('cone_mapper')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create cone detection subscriber
        self.cones_subscription = self.create_subscription(
            Cones,
            'cone_detection',
            self.cones_callback,
            qos_profile)
        
        # Create car position subscriber
        self.car_subscription = self.create_subscription(
            Pose,
            'car_position',
            self.car_position_callback,
            qos_profile)
        
        # Store car positions
        self.car_positions = []

        # Create cone map publisher
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)

        # Cone types
        self.blue = 0
        self.yellow = 2

        # Existing cone map
        self.Cone_map = None

        # KDTrees for searching
        self.left_tree = None
        self.right_tree = None

################################################################################ (parameters to tune)

        # Initial error in the estimate
        self.default_error_in_estimate = 5.0

        # Error in measurement
        self.error_in_measurement = 2.5

        # Cone match radius
        self.match_radius = 0.1

################################################################################ (parameters to tune)

    def car_position_callback(self, msg: Pose) -> None:
        """This function receives the car's current position

        Args:
            msg (Pose): input car position from the /car_position topic
        """
        self.car_positions.append(msg)


    def cones_callback(self, msg: Cones) -> None:
        """This function updates the existing cone map and publish it to the /cone_map topic

        Args:
            msg (Cones): input cones from the /cone_detection topic
        """
        # Get the car's current position
        while len(self.car_positions) == 0:
            pass
        car_position = self.car_positions.pop(0)

        # Update the existing cone map with new measurements
        self.update_existing_cone_map(msg, car_position)

        # Publishes the existing cone map to the /cone_map topic
        self.publisher.publish(self.Cone_map)


    def update_existing_cone_map(self, msg: Cones, car_position: Pose) -> None:
        """This function updates the existing cone map using new cone measurements

        Args:
            msg (Cones): A list of detected cones
            car_position (Pose): car's current position
        """
        # Get global measurements
        global_cone_columns = self.cones_local_to_global(msg, car_position)

        # Collect all cone's (x, y)
        list_of_cones_x, list_of_cones_y, list_of_cones_type = global_cone_columns
        left_points = [(list_of_cones_x[i], list_of_cones_y[i]) for i in range(len(list_of_cones_x)) if list_of_cones_type[i] == self.blue]
        right_points = [(list_of_cones_x[i], list_of_cones_y[i]) for i in range(len(list_of_cones_x)) if list_of_cones_type[i] == self.yellow]

        # Update the existing cone map
        if self.Cone_map == None:
            # If the existing cone map is empty, pack all cone measurements and update the existing cone map
            self.Cone_map = self.produce_cone_map_message(left_points, right_points)
            self.left_tree = kdtree.create([Item(coord[0], coord[1], (index, self.default_error_in_estimate)) for index, coord in enumerate(left_points)])
            self.right_tree = kdtree.create([Item(coord[0], coord[1], (index, self.default_error_in_estimate)) for index, coord in enumerate(right_points)])
        else:
            # If the existing cone map is not empty, find the closest cone for each newly measured cone and update its coordinates or add it to the map
            self.update_left_track(left_points)
            self.update_right_track(right_points)


    def update_left_track(self, points: list) -> None:
        """This function updates the left track of the cone map

        Args:
            points (list): A list of blue cones' (x, y) coordinates
        """
        for coord in points:
            point, distance = self.left_tree.search_nn(coord)
            # If the newly measured cone is in the match radius, this cone already exists in the existing cone map, update its coordinates
            if distance <= self.match_radius:
                index = point.data.data[0]
                cone = self.Cone_map.left_cones[index]
                error_in_estimate = point.data.data[1]
                new_estimate_x, new_estimate_y, new_error_in_estimate = self.kalman_filtering(cone, coord, error_in_estimate)
                point.data.coords = (new_estimate_x, new_estimate_y)
                point.data.data[1] = new_error_in_estimate
            else:
                # If the newly measured cone is not in the match radius, this cone does not exist in the existing cone map, add it to the existing cone map
                cone = Point()
                cone.x = coord[0]
                cone.y = coord[1]
                self.Cone_map.left_cones.append(cone)
                self.left_tree.add(Item(coord[0], coord[1], (len(self.Cone_map.left_cones) - 1, self.default_error_in_estimate)))
    

    def update_right_track(self, points: list) -> None:
        """This function updates the right track of the cone map

        Args:
            points (list): A list of yellow cones' (x, y) coordinates
        """
        for coord in points:
            point, distance = self.right_tree.search_nn(coord)
            # If the newly measured cone is in the match radius, this cone already exists in the existing cone map, update its coordinates
            if distance <= self.match_radius:
                index = point.data.data[0]
                cone = self.Cone_map.right_cones[index]
                error_in_estimate = point.data.data[1]
                new_estimate_x, new_estimate_y, new_error_in_estimate = self.kalman_filtering(cone, coord, error_in_estimate)
                point.data.coords = (new_estimate_x, new_estimate_y)
                point.data.data[1] = new_error_in_estimate
            else:
                # If the newly measured cone is not in the match radius, this cone does not exist in the existing cone map, add it to the existing cone map
                cone = Point()
                cone.x = coord[0]
                cone.y = coord[1]
                self.Cone_map.right_cones.append(cone)
                self.right_tree.add(Item(coord[0], coord[1], (len(self.Cone_map.left_cones) - 1, self.default_error_in_estimate)))

    
    def kalman_filtering(self, cone: Point, coord: tuple, error_in_estimate: float) -> tuple:
        """This function update the x and y coordinate of of the input cone using kalman filtering

        Args:
            cone (Point): input cone's location
            coord (tuple): new measurement of (x, y) of the input cone
            error_in_estimate (float): error in estimate of the cone's actual position
        
        Returns:
            new_estimate_x (float): cone's new estimated x coordinate
            new_estimate_y (float): cone's new estimated y coordinate
            new_error_in_estimate (float): cone's new error in estimate
        """
        # Get the previous error_in_estimate, estimated x and estimated y coordinate
        prev_error_in_estimate = error_in_estimate
        prev_estimate_x = cone.x
        prev_estimate_y = cone.y

        # Calculate the kalman gain
        kalman_gain = prev_error_in_estimate / (prev_error_in_estimate + self.error_in_measurement)

        # Calculate the new error_in_estimate, estimated x and estimated y coordinate
        new_estimate_x = prev_estimate_x + kalman_gain * (coord[0] - prev_estimate_x)
        new_estimate_y = prev_estimate_y + kalman_gain * (coord[1] - prev_estimate_y)
        new_error_in_estimate = (1 - kalman_gain) * (prev_error_in_estimate)

        # update x and y coordinate
        cone.x = new_estimate_x
        cone.y = new_estimate_y

        return new_estimate_x, new_estimate_y, new_error_in_estimate


    def cones_local_to_global(self, msg: Cones, car_position: Pose) -> np.array:
        """This function converts all cones from the local frame to the global frame

        Args:
            msg (Cones): contains all the measured cones in local frame
            car_position (Pose): car's current position

        Returns:
            global_cone_columns (np.array): 3*n np.array contains each cone's global x coordinate, y coordinate and type
        """
        # Extract car's x coordinate, y coordinate and rotation angle
        x, y, theta = self.extract_data_from_car(car_position)

        # Convert cones message into a np.array contains each cone's local x coordinate, y coordinate and type
        list_of_cones = self.convert_cones_to_data(msg)

        # Use cart's (x, y and theta) to get the cart's position vector and rotation matrix
        position_vector, rotation_matrix = self.get_coordinate_conversion_matrices(x, y, theta)

        # Convert cone's coordinates from local frame to global frame
        global_cone_columns = self.local_to_global(position_vector, rotation_matrix, list_of_cones)

        return global_cone_columns
    

    def extract_data_from_car(self, car_position: Pose) -> tuple:
        """This function extracts car's x coordinate, y coordinate and rotation angle from car_position

        Args:
            car_position (Pose): car's current position

        Returns:
            x (float): car's x coordinate
            y (float): car's y coordinate
            theta (float): car's rotation angle
        """
        x = car_position.position.x
        y = car_position.position.y
        theta = car_position.orientation.w
        return x, y, theta


    def convert_cones_to_data(self, msg: Cones) -> np.array:
        """Convert cones message into a 3*n np.array contains each cone's local x coordinate, y coordinate and type

        Args:
            msg (Cones): cones message

        Returns:
            list_of_cones (np.array): 3*n np.array contains each cone's local x coordinate, y coordinate and type
        """
        list_of_cones = msg.cones

        # Store cones' data
        list_of_local_cones_x = []
        list_of_local_cones_y = []
        list_of_local_cones_type = []

        # Extract data from all cone messages and store them in a 3*n np.array
        for cone in list_of_cones:
            individual_x, individual_y, individual_type = self.extract_data_from_cone(cone)
            list_of_local_cones_x.append(individual_x)
            list_of_local_cones_y.append(individual_y)
            list_of_local_cones_type.append(individual_type)

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y, list_of_local_cones_type])
                
        return list_of_cones
    

    def extract_data_from_cone(self, cone_input: Cone) -> tuple:
        """This function extracts data from the cone message

        Args:
            cone_input (Cone): cone message

        Returns:
            x (float): cone's x coordinate
            y (float): cone's y coordinate
            type (int): cone's type
        """
        x = cone_input.position.x
        y = cone_input.position.y
        cone_type = cone_input.type
        return x, y, cone_type
    

    def get_coordinate_conversion_matrices(self, x: float, y: float, theta: float) -> tuple:
        """This function gets car's gloabl position vector and local coordinate rotation matrix

        Args:
            x (float): car's x coordinate
            y (float): car's y coordinate
            theta (float): car's rotation angle

        Returns:
            position_vector (np.array): car's global position vector
            rotation_matrix (np.array): rotation matrix for cone's local coordinates
        """
        position_vector = np.array([[x],[y],[0]])
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0],[math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

        return position_vector, rotation_matrix
    

    def local_to_global(self, position_vector: np.array, rotation_matrix: np.array, list_of_cones: np.array) -> np.array:
        """This function converts all cones data (x, y, type) from the local frame to the global frame

        Args:
            position_vector (np.array): car's global position vector
            rotation_matrix (np.array): rotation matrix for cone's local coordinates
            list_of_cones (np.array): 3*n np.array contains each cone's local x coordinate, y coordinate and type

        Returns:
            list_of_cones_output (np.array): 3*n np.array contains each cone's global x coordinate, y coordinate and type
        """
        list_of_cones_unrotated = np.matmul(rotation_matrix, list_of_cones)
        list_of_cones_output = list_of_cones_unrotated + position_vector
        return list_of_cones_output
    

    def produce_cone_map_message(self, left_points: list, right_points: list) -> ConeMap:
        """This function packs all cones' (x, y) coordinates into a cone map message

        Args:
            left_points (list): A list of blue cones' (x, y) coordinates
            right_points (list): A list of yellow cones' (x, y) coordinates

        Returns:
            output_map (ConeMap): cone map message
        """
        output_map = ConeMap()
        for lp in left_points:
            point = Point()
            point.x = lp[0]
            point.y = lp[1]
            output_map.left_cones.append(point)

        for rp in right_points:
            point = Point()
            point.x = rp[0]
            point.y = rp[1]
            output_map.right_cones.append(point)

        return output_map


def main(args=None):
    rclpy.init(args=args)
    cone_mapper = Cone_Mapper()
    rclpy.spin(cone_mapper)
    cone_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
