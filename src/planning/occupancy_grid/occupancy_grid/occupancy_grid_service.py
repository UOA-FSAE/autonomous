import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import cv2 as cv
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from mapping_interfaces.msg import ConeMap
from mapping_interfaces.msg import Cone
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Occupancy_grid(Node):

    #Example codes
    def __init__(self):
        super().__init__('Occupancy_grid')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
    
    def add_two_ints_callback(self, request, response):
        print("Testing occupancy grid function");
        print(self.populating_occupancy_grid());
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

        #Occupancy grid has got row to x and column to y

        # interpolate between cones
        self.interpolate_bound(occupancy_grid_matrix, x_list, y_list)

        bodyfit_adj = self.bodyfit_adjust(occupancy_grid_matrix)

        left, right = self.separate_l_r(bodyfit_adj)

        self.plot_occupancy_grid(x_list, y_list, occupancy_grid_matrix);
        #Interpolate between cones to get full occupancy grid
        #Zane~~~~~

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
        for cone in cone_map.cones:
            x,y = self.find_cone_coordinate(x_list, y_list, cone);
            occupancy_grid_matrix[x][y] = 255;

    def get_matrix_from_size(self, x1, x2, y1, y2):
        resolution = 0.1;  # In meters
        width = int(math.ceil((x2 - x1) / resolution));
        height = int(math.ceil((y2 - y1) / resolution));
        matrix_output = np.empty((width, height), dtype = float);
        x_list = np.arange(x1, x2, resolution);
        y_list = np.arange(y1, y2, resolution);
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
        cone1.pose.pose.position.x = -1.0;
        cone2.pose.pose.position.x = -1.0;
        cone3.pose.pose.position.x = -1.0;
        cone4.pose.pose.position.x = -1.0;
        cone5.pose.pose.position.x = 1.0;
        cone6.pose.pose.position.x = 1.0;
        cone7.pose.pose.position.x = 1.0;
        cone8.pose.pose.position.x = 1.0;
        cone1.pose.pose.position.y = 4.0;
        cone2.pose.pose.position.y = 2.0;
        cone3.pose.pose.position.y = -2.0;
        cone4.pose.pose.position.y = -4.0;
        cone5.pose.pose.position.y = 4.0;
        cone6.pose.pose.position.y = 2.0;
        cone7.pose.pose.position.y = -2.0;
        cone8.pose.pose.position.y = -4.0;
        cone_map_test.cones.append(cone1);
        cone_map_test.cones.append(cone2);
        cone_map_test.cones.append(cone3);
        cone_map_test.cones.append(cone4);
        cone_map_test.cones.append(cone5);
        cone_map_test.cones.append(cone6);
        cone_map_test.cones.append(cone7);
        cone_map_test.cones.append(cone8);
        return cone_map_test

    def get_map_size(self, cone_map:ConeMap):
        #Get maximum andd minimum x and y
        x1 = 0;
        x2 = 0;
        y1 = 0;
        y2 = 0;
        for cone in cone_map.cones:
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

    def plot_occupancy_grid(self, x_list, y_list, occupancy_grid_matrix):
        #Transpose it for having column to x and row to y:
        occupancy_grid_matrix = np.transpose(occupancy_grid_matrix);

        #Grid plotting
        X,Y = np.meshgrid(x_list, y_list);
        Z = np.sin(np.sqrt(X ** 2 + Y ** 2))
        print(Z.shape);
        print(occupancy_grid_matrix.shape);

        # Create a figure and a 3D axis
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Plot the surface
        surface = ax.plot_surface(X, Y, occupancy_grid_matrix, cmap=plt.cm.coolwarm)

        # Add color bar
        fig.colorbar(surface, shrink=0.5, aspect=10)

        # Set labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Cone appearance')

        # Show the plot
        plt.show()

    ##############################Boundary mapping
    def interpolate_bound(self, occ_grid : np.ndarray, x_list, y_list):
        spline_tck, u = splprep([x_list,y_list], s=175, per=True) # can modify s (possibly need to tune parameter)
        
        #evaluate spline at given point
        xi, yi = splev(u, spline_tck)

        # plot the result
        fig, ax = plt.subplots(1, 1)
        ax.plot(x_list, y_list, 'or')
        ax.plot(xi, yi, '-b')
        plt.show()

        return 

    def bodyfit_adjust(self, occ_grid : np.ndarray) -> np.ndarray:
        '''
            Inputs: 
            occ_grid = Occupancy grid with interpolated boundary
            resolution = distance per pixel

            Output:
            - shrunken_track = Occupancy grid that depicts drivable area and accounts 
            for width of the car
        '''
        
        shrinkage_amount = 15 # TODO need to account for resolution to check how many pixels to shrink by
        kernel_size = (shrinkage_amount // 2, shrinkage_amount // 2)
        kernel = np.ones(kernel_size, np.uint8)

        shrunken_track = cv.erode(occ_grid, kernel)
        
        return shrunken_track

    def separate_l_r(occ_grid : np.ndarray):
        '''
            Inputs:
            occ_grid = Eroded occupancy grid 

            Output:
            - left = list of arrays containing points of left boundary
            - right = list of arrays containing points of right boundary
        '''
        #separate inside from outside
        contours, hierarchy = cv.findContours(occ_grid, cv.RETR_CCOMP, cv.CHAIN_APPROX_NONE)

        new = cv.cvtColor(occ_grid, cv.COLOR_GRAY2BGR)

        left, right = [],[]

        for i in range(len(contours)):
            # Draw the contour
            if hierarchy[0][i][3] == -1:
                # Outer contour, draw in red
                cv.drawContours(new, contours, i, (0, 0, 255), 1)

                # Outer contour, store its points
                right.append(contours[i])
            else:
                # Inner contour, draw in blue
                cv.drawContours(new, contours, i, (255, 0, 0), 1)

                # Inner contour, store its points
                left.append(contours[i])

        cv.imshow('contours', new) # for testing 
        
        #clean boundary arrays
        left = [inner.flatten() for inner in np.vstack(left)]
        right = [inner.flatten() for inner in np.vstack(right)]

        left, right = np.array(left), np.array(right)

        return left, right




def main():
    rclpy.init()

    minimal_service = Occupancy_grid()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()