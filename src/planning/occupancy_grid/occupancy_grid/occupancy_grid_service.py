from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from mapping_interfaces import ConeMap
from mapping_interfaces inport Cone
import math
import numpy as np


class Occupancy_grid(Node):

    #Example codes
    def __init__(self):
        super().__init__('Occupancy_grid')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
    
    #Actual coding happening from here
    ##############################Populating occupancy grid
    def populating_occupancy_grid(self):
        #Debug only: create a custom cone map
        #To do: get cone map from cone_mapping service
        cone_map = self.generate_cone_map_for_test(); #Generate dataset for test

        #Get cone map size
        x1, x2, y1, y2 = self.get_map_size(cone_map);

        #Initialize matrix with size of map
        occupancy_grid_matrix, x_list, y_list = self.get_matrix_from_size(x1, x2, y1, y2);

        #Put high integer value on cone occupancy grid
        self.fill_occupancy_grid(cone_map, occupancy_grid_matrix, x_list, y_list)

        #Interpolate between cones to get full occupancy grid
        #Output
        return occupancy_grid_matrix;

    def binary_search(self, x_list, x):
        total_length = len(x_list);
        # print(total_length);
        start_index = 0;
        end_index = total_length - 1;
        mid_index = (start_index + end_index) // 2;
        while not (start_index == mid_index or mid_index == end_index):
            # print(start_index);
            # print(mid_index);
            # print(end_index);
            # print("------")
            if x >= x_list[mid_index]:
                start_index = mid_index;
                mid_index = (start_index + end_index) // 2
            elif x < x_list[mid_index]:
                end_index = mid_index;
                mid_index = (start_index + end_index) // 2

        # Select box that contains the value inclusively
        return start_index;

    def find_cone_coordinate(self, x_list, y_list, cone):
        x, y, theta, covariance = self.extract_data_from_cone(cone)
        x = self.binary_search(x_list, x);
        y = self.binary_search(y_list, y);
        return x,y

    def fill_occupancy_grid(self, cone_map, occupancy_grid_matrix, x_list, y_list):
        for cone in cone_map.Cones:
            x,y = self.find_cone_coordinate(x_list, y_list, cone);
            occupancy_grid_matrix[x][y] = 255;

    def get_matrix_from_size(self, x1, x2, y1, y2):
        resolution = 0.1;  # In meters
        width = int(math.ceil((x2 - x1) / resolution));
        height = int(math.ceil((y2 - y1) / resolution));
        matrix_output = np.empty(width, height);
        x_list = range(x1, x2, resolution);
        y_list = range(y1, y2, resolution);
        return matrix_output, x_list, y_list;

    def generate_cone_map_for_test(self):
        #For debug only: Generate a 10 x 10 cone:
        cone_map_test = ConeMap();
        cone1 = Cone();
        cone2 = Cone();
        cone3 = Cone();
        cone4 = Cone();
        cone5 = Cone();
        cone6 = Cone();
        cone7 = Cone();
        cone8 = Cone();
        cone1.pose.pose.position.x = -1;
        cone2.pose.pose.position.x = -1;
        cone3.pose.pose.position.x = -1;
        cone4.pose.pose.position.x = -1;
        cone5.pose.pose.position.x = 1;
        cone6.pose.pose.position.x = 1;
        cone7.pose.pose.position.x = 1;
        cone8.pose.pose.position.x = 1;
        cone1.pose.pose.position.y = 4;
        cone2.pose.pose.position.y = 2;
        cone3.pose.pose.position.y = -2;
        cone4.pose.pose.position.y = -4;
        cone5.pose.pose.position.y = 4;
        cone6.pose.pose.position.y = 2;
        cone7.pose.pose.position.y = -2;
        cone8.pose.pose.position.y = -4;
        cone_map_test.Cones.append(cone1);
        cone_map_test.Cones.append(cone2);
        cone_map_test.Cones.append(cone3);
        cone_map_test.Cones.append(cone4);
        cone_map_test.Cones.append(cone5);
        cone_map_test.Cones.append(cone6);
        cone_map_test.Cones.append(cone7);
        cone_map_test.Cones.append(cone8);
        return cone_map_test

    def get_map_size(self, cone_map:ConeMap):
        #Get maximum andd minimum x and y
        x1 = 0;
        x2 = 0;
        y1 = 0;
        y2 = 0;
        for cone in cone_map.Cones:
            x, y, theta, covariance = self.extract_data_from_cone(cone)
            if x < x1:
                x1 = x;
            elif x > x2:
                x2 = x;
            if y < y1:
                y1 = y;
            elif y > y2:
                y2 = y;

        return x1, x2, y1, y2

    def extract_data_from_cone(self, cone_input : Cone) -> (float, float, float, list[float]):
        x = cone_input.pose.pose.position.x;
        y = cone_input.pose.pose.position.y;
        theta = cone_input.pose.pose.orientation.w;
        covaraince = cone_input.pose.covariance;
        return x, y, theta, covaraince

    ##############################Boundary mapping


    ##############################Bodyfit adjustment


    ##############################Driveable region decision



def main():
    rclpy.init()

    minimal_service = Occupancy_grid()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()