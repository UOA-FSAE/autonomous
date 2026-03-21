"""
Cone Landmark Mapper — maintains a persistent map of cone positions on the track.

Subscribes to /cone_detection (fsae_interfaces/Detections) which provides the
car's pose and lists of blue/yellow cones in the camera's local frame.

For each detection frame:
  1. Transforms cone positions from the car's local frame to the global frame.
  2. Matches each observed cone to the nearest known landmark using a KD-tree
     (within a configurable match radius).
  3. If matched: refines the landmark's position via a 1-D Kalman filter and
     increments a confidence counter.
  4. If unmatched: inserts a new landmark into the map.
  5. Periodically prunes low-confidence landmarks (those not seen often enough).

Publishes:
  /left_track  (fsae_interfaces/Track) — refined blue cone positions
  /right_track (fsae_interfaces/Track) — refined yellow cone positions
  /times_modified (Float32MultiArray)  — confidence scores of pruned cones (debug)

Parameters:
  invert_cones (bool, default False) — swap blue↔yellow track assignment
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32MultiArray
from fsae_interfaces.msg import Detections, Track
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance

import math
import numpy as np
import time
import threading

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
    def __init__(self, *vargs, **kwargs):
        super().__init__('cone_mapper', *vargs, **kwargs)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create cone detection subscriber
        self.cones_subscription = self.create_subscription(
            Detections,
            'cone_detection',
            self.cones_callback,
            qos_profile)

        # Create left track publisher
        self.left_track_publisher = self.create_publisher(Track, 'left_track', 10)

        # Create right track publisher
        self.right_track_publisher = self.create_publisher(Track, 'right_track', 10)

        # times_modified publisher
        self.times_modified_publisher = self.create_publisher(Float32MultiArray, 'times_modified', 10)

        # Existing cone map
        self.left_track = Track()
        self.left_track.cones = []
        self.right_track = Track()
        self.right_track.cones = []

        # KDTrees for searching
        self.left_tree = kdtree.create(None, dimensions=2)
        self.right_tree = kdtree.create(None, dimensions=2)

        self.times_updated_counter = 0

################################################################################ (parameters to tune)

        # Initial error in the estimate
        self.default_error_in_estimate = 4.0

        # Error in measurement
        self.error_in_measurement = 2.5

        # Cone match radius
        self.match_radius = 1.0

        # Counter to remove points
        self.remove_point_counter = 200

        # value for the times_modified
        self.times_modified_limit = 100

        # Modify rate
        self.modify_rate = 1.08
        
        # Detection reset counter
        self.reset_counter = 0
        self.reset_count = 60

        self.declare_parameters(
            namespace='',
            parameters=[
            ('invert_cones', False),
            ]
        )

################################################################################
#(parameters to tune)

    def reset_map(self):
        self.left_track = Track()
        self.left_track.cones = []
        self.right_track = Track()
        self.right_track.cones = []

        # KDTrees for searching
        self.left_tree = kdtree.create(None, dimensions=2)
        self.right_tree = kdtree.create(None, dimensions=2)

    def cones_callback(self, msg: Detections) -> None:
        """This function updates the existing cone map with newly detected cones and 
        publishes left-track cones to the /left_track topic and right-track cones to the /right_track topic.

        Args:
            msg (Detections): car's position and input cones from the /cone_detection topic
        """
        # Update the existing cone map with new measurements
        self.update_existing_cone_map(msg)

        self.times_updated_counter += 1
        if self.times_updated_counter > self.remove_point_counter:
            self.times_updated_counter = 0
            self.remove_cones()
        
        self.reset_counter += 1
        if self.reset_counter > self.reset_count:
            self.reset_counter = 0
            self.reset_map()
        
        self.left_track_publisher.publish(self.left_track)
        self.right_track_publisher.publish(self.right_track)


    def update_existing_cone_map(self, msg: Detections) -> None:
        """This function updates the existing cone map using new cone measurements

        Args:
            msg (Detections): Contains the car's position and lists of detected cones
        """
        # Get all cones' global positions
        car_position = msg.car_pose
        global_blue_cone_positions = self.cones_local_to_global(msg.blue, car_position)
        global_yellow_cone_positions = self.cones_local_to_global(msg.yellow, car_position)

        # Update the left track and the right track if cones got detected
        if not self.get_parameter('invert_cones').get_parameter_value().bool_value:
            if len(global_blue_cone_positions) != 0:
                self.update_left_track(global_blue_cone_positions)
            if len(global_yellow_cone_positions) != 0:
                self.update_right_track(global_yellow_cone_positions)
        else:
            if len(global_yellow_cone_positions) != 0:
                self.update_left_track(global_yellow_cone_positions)
            if len(global_blue_cone_positions) != 0:
                self.update_right_track(global_blue_cone_positions)


    def update_left_track(self, points: list) -> None:
        """This function updates the left track of the cone map

        Args:
            points (list): A list of blue cones' global (x, y) coordinates
        """
        if len(self.left_track.cones) == 0:
            # If the left track is empty, pack all cone measurements and update the left track
            self.add_cones_to_left_track(points)
            self.left_tree = kdtree.create([Item(coord[0], coord[1], [self.left_track.cones[index], self.default_error_in_estimate, 1]) for index, coord in enumerate(points)]) # !!!!!!
        else:
            # If the left track is not empty, find the closest cone for each newly measured cone and update its coordinates or add it to the track
            for coord in points:
                point, distance = self.left_tree.search_nn(coord)
                # If the newly measured cone is in the match radius, this cone already exists in the left track, update its coordinates
                if distance <= self.match_radius:
                    cone = point.data.data[0]
                    error_in_estimate = point.data.data[1]
                    times_modified = point.data.data[2] * self.modify_rate
                    new_estimate_x, new_estimate_y, new_error_in_estimate = self.kalman_filtering(cone, coord, error_in_estimate)
                    point.data.coords = (new_estimate_x, new_estimate_y)
                    point.data.data[1] = new_error_in_estimate
                    point.data.data[2] = times_modified
                # If the newly measured cone is not in the match radius, this cone does not exist in the existing cone map, add it to the existing cone map
                else:
                    cone = Point()
                    cone.x = coord[0]
                    cone.y = coord[1]
                    self.left_track.cones.append(cone)
                    self.left_tree.add(Item(coord[0], coord[1], [cone, self.default_error_in_estimate, 1]))

    

    def update_right_track(self, points: list) -> None:
        """This function updates the right track of the cone map

        Args:
            points (list): A list of blue cones' global (x, y) coordinates
        """
        if len(self.right_track.cones) == 0:
            # If the right track is empty, pack all cone measurements and update the right track
            self.add_cones_to_right_track(points)
            self.right_tree = kdtree.create([Item(coord[0], coord[1], [self.right_track.cones[index], self.default_error_in_estimate, 1]) for index, coord in enumerate(points)]) # !!!!!!
        else:
            # If the right track is not empty, find the closest cone for each newly measured cone and update its coordinates or add it to the track
            for coord in points:
                point, distance = self.right_tree.search_nn(coord)
                # If the newly measured cone is in the match radius, this cone already exists in the right track, update its coordinates
                if distance <= self.match_radius:
                    cone = point.data.data[0]
                    error_in_estimate = point.data.data[1]
                    times_modified = point.data.data[2] * self.modify_rate
                    new_estimate_x, new_estimate_y, new_error_in_estimate = self.kalman_filtering(cone, coord, error_in_estimate)
                    point.data.coords = (new_estimate_x, new_estimate_y)
                    point.data.data[1] = new_error_in_estimate
                    point.data.data[2] = times_modified
                # If the newly measured cone is not in the match radius, this cone does not exist in the existing cone map, add it to the existing cone map
                else:
                    cone = Point()
                    cone.x = coord[0]
                    cone.y = coord[1]
                    self.right_track.cones.append(cone)
                    self.right_tree.add(Item(coord[0], coord[1], [cone, self.default_error_in_estimate, 1]))

    
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


    def cones_local_to_global(self, cones: list, car_position: Pose) -> list:
        """This function converts all cones from the local frame to the global frame

        Args:
            cones ([Point]): contains all the cones' positions in local frame
            car_position (Pose): car's current position

        Returns:
            global_cone_positions (list): a list contains each cone's global (x, y)
        """
        # If no cones are detected, return an empty np.array
        if len(cones) == 0:
            return []
        
        # Extract car's x coordinate, y coordinate and rotation angle
        x, y, theta = self.extract_data_from_car(car_position)

        # Convert cones positions into a np.array contains each cone's local x coordinate, y coordinate
        list_of_cones = self.convert_cones_to_data(cones)

        # Use cart's (x, y and theta) to get the cart's position vector and rotation matrix
        position_vector, rotation_matrix = self.get_coordinate_conversion_matrices(x, y, theta)

        # Convert cone's coordinates from local frame to global frame
        global_cone_columns = self.local_to_global(position_vector, rotation_matrix, list_of_cones)

        # Convert global_cone_columns to a list of cones' (x,y) position
        global_cone_positions = [tuple(global_cone_columns[:,i]) for i in range(len(cones))]

        return global_cone_positions
    

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


    def convert_cones_to_data(self, cones: list) -> np.array:
        """Convert a list of cones' positions into a 2*n np.array contains each cone's local x coordinate and y coordinate

        Args:
            cones ([Point]): A list of cones, each cone is represented as a Point

        Returns:
            list_of_cones (np.array): 2*n np.array contains each cone's local x coordinate and y coordinate
        """
        # Extract x and y coordinate from all cones and store them in a 2*n np.arra
        list_of_local_cones_x = [cone.x for cone in cones]
        list_of_local_cones_y = [cone.y for cone in cones]
        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y])       
        return list_of_cones
 

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
        position_vector = np.array([[x],[y]])
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])

        return position_vector, rotation_matrix
    

    def local_to_global(self, position_vector: np.array, rotation_matrix: np.array, list_of_cones: np.array) -> np.array:
        """This function converts all cones' (x, y) from the local frame to the global frame

        Args:
            position_vector (np.array): car's global position vector
            rotation_matrix (np.array): rotation matrix for cone's local coordinates
            list_of_cones (np.array): 2*n np.array contains each cone's local x coordinate and y coordinate

        Returns:
            list_of_cones_output (np.array): 2*n np.array contains each cone's global x coordinate and y coordinate
        """
        list_of_cones_unrotated = np.matmul(rotation_matrix, list_of_cones)
        list_of_cones_output = list_of_cones_unrotated + position_vector
        return list_of_cones_output
    

    def add_cones_to_left_track(self, points: list) -> None:
        """This function packs all cones' (x, y) coordinates and add them to the left track

        Args:
            points (list): A list of cones' (x, y) coordinates
        """
        for p in points:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            self.left_track.cones.append(point)

    def add_cones_to_right_track(self, points: list) -> None:
            """This function packs all cones' (x, y) coordinates and add them to the right track

            Args:
                points (list): A list of cones' (x, y) coordinates
            """
            for p in points:
                point = Point()
                point.x = p[0]
                point.y = p[1]
                self.right_track.cones.append(point)

    def remove_cones(self):
        if len(self.left_track.cones) == 0 or len(self.right_track.cones) == 0:
            return
        
        times_modified_list = Float32MultiArray()
        
        for point in kdtree.level_order(self.left_tree):
            if point.data.data[2] < self.times_modified_limit:
                times_modified_list.data.append(point.data.data[2])
                self.left_track.cones.remove(point.data.data[0])
                self.left_tree = self.left_tree.remove(point.data)
                
        for point in kdtree.level_order(self.right_tree):
            if point.data.data[2] < self.times_modified_limit:
                times_modified_list.data.append(point.data.data[2])
                self.right_track.cones.remove(point.data.data[0])
                self.right_tree = self.right_tree.remove(point.data)
        
        self.times_modified_publisher.publish(times_modified_list)
        

from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    ctx = Context()
    rclpy.init(args=args, context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    cone_mapper = Cone_Mapper(context=ctx)
    executor.add_node(cone_mapper) 
    
    try:
        executor.spin() # shuts down internally on SIGINT signal
    except KeyboardInterrupt:
        cone_mapper.get_logger().info("keyboard interrupt signal intercepted")
    finally:
        cone_mapper.get_logger().info("shutting down")
        
    cone_mapper.destroy_node()


if __name__ == '__main__':
    main()
