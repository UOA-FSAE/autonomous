import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import cv2 as cv
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone
# from moa_msgs.msg import OccupancyGrid
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Occupancy_grid(Node):
    def __init__(self):
        super().__init__('Occupancy_grid')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        
        # self.occ_grid_publisher = self.create_publisher(
        #     OccupancyGrid,
        #     'track/occ_grid',
        #     10
        # )

        # self.bound_l_publisher = self.create_publisher(
        #     OccupancyGrid,
        #     'track/bound_l',
        #     10
        # )

        # self.bound_r_publisher = self.create_publisher(
        #     OccupancyGrid,
        #     'track/bound_r',
        #     10
        # )


        # self.subscription = self.create_subscription(
        #     ConeMap,
        #     'cone_map',
        #     self.publish_occ_grid,
        #     10
        # )

        self.resolution = 1 # in m/pixel TODO fix for resolutions other than 1
        self.car_width = 1.5 # in m TODO change to acutal width

    def add_two_ints_callback(self, request, response):
        print("Testing occupancy grid function")
        cone_map = self.generate_cone_map_for_test()
        print(self.gen_occ_grid(cone_map))
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
    
    # def publish_occ_grid(self, msg):
    #     bound_l, bound_r, occ_grid = self.gen_occ_grid(msg)
    #     msg = OccupancyGrid()

    #     msg.occupancyGrid = occ_grid
    #     self.occ_grid_publisher.publish(msg)     
        
    #     msg.occupancyGrid = bound_l
    #     self.bound_l_publisher.publish(msg) 
        
    #     msg.occupancyGrid = bound_r
    #     self.bound_r_publisher.publish(msg) 

    def gen_occ_grid(self, cone_map):
        # #Debug only: generate test cone map
        # #To do: get cone map from cone_mapping service
        # cone_map = self.generate_cone_map_for_test() #Generate dataset for test
        self.start_pos = cone_map.cones[0]
        cone_map = cone_map.cones[1:]
        
        #Get cone map size
        max_x, min_x, max_y, min_y = self.get_map_boundary(cone_map)

        #Initialize matrix with size of map
        occupancy_grid_matrix, x_list, y_list = self.create_matrix(max_x, min_x, max_y, min_y)

        #Put high integer value on cone occupancy grid
        cones_l, cones_r = self.fill_occupancy_grid(cone_map, occupancy_grid_matrix, x_list, y_list)
        # cv.imwrite('occ_grid.png', occupancy_grid_matrix)
        
        # interpolate between cones
        self.interpolate_bounds(occupancy_grid_matrix, cones_l, cones_r, x_list, y_list)
        # cv.imwrite('interpolated.png', occupancy_grid_matrix)

        # apply bodyfit adjustment
        bodyfit_adj = self.bodyfit_adjust(occupancy_grid_matrix, self.car_width)
        # cv.imwrite('shrunken.png', bodyfit_adj)

        # extract left and right edges from occupancy grid
        adj_bound_l, adj_bound_r = self.sep_l_r_bounds(bodyfit_adj)

        self.plot_occupancy_grid(x_list, y_list, occupancy_grid_matrix)

        return bodyfit_adj, adj_bound_l, adj_bound_r

    def binary_search(self, x_list, x):
        total_length = len(x_list)
        # print(total_length)
        start_index = 0
        end_index = total_length - 1
        mid_index = (start_index + end_index) // 2
        while not (start_index == mid_index or mid_index == end_index):
            # print(start_index)
            # print(mid_index)
            # print(end_index)
            # print("------")
            if x >= x_list[mid_index]:
                start_index = mid_index
                mid_index = (start_index + end_index) // 2
            elif x < x_list[mid_index]:
                end_index = mid_index
                mid_index = (start_index + end_index) // 2

        # Select box that contains the value inclusively
        return start_index

    def find_cone_coordinate(self, x_list, y_list, cone):
        cone_x, cone_y, theta, covariance, colour = self.extract_cone_data(cone)
        mapped_x = self.binary_search(x_list, cone_x)
        mapped_y = self.binary_search(y_list, cone_y)

        return  mapped_x, mapped_y, colour

    def fill_occupancy_grid(self, cone_map, occupancy_grid_matrix, x_list, y_list):
        bound_l, bound_r = [], []

        for cone in cone_map:
            x, y, colour = self.find_cone_coordinate(x_list, y_list, cone)
            occupancy_grid_matrix[y][x] = 255
        
            if not colour:
                bound_l.append(np.array((x,y)))
            else:
                bound_r.append(np.array((x,y)))
        
        return np.array(bound_l), np.array(bound_r)
                
    def create_matrix(self, max_x, min_x, max_y, min_y):
        mat_width = math.ceil((max_x - min_x) / self.resolution)
        mat_height = math.ceil((max_y - min_y) / self.resolution)
        
        matrix_output = np.zeros((mat_height, mat_width), dtype = np.uint8)
        
        x_list = np.arange(min_x, max_x, self.resolution)
        y_list = np.arange(min_y, max_y, self.resolution)
        return matrix_output, x_list, y_list

    def generate_cone_map_for_test(self):
        #For debug only: Generate a 10 x 10 cone:
        cone_map_test = ConeMap()

        left_arr = np.loadtxt('src/planning/occupancy_grid/occupancy_grid/testcase_L.csv', delimiter=',')
        right_arr = np.loadtxt('src/planning/occupancy_grid/occupancy_grid/testcase_R.csv', delimiter=',')
        
        cone = Cone()
        cone.pose.pose.position.x = 52.0
        cone.pose.pose.position.y = 3.0
        cone.colour = 0
        cone_map_test.cones.append(cone)

        for pt in left_arr:
            cone = Cone()
            cone.pose.pose.position.x = pt[0]
            cone.pose.pose.position.y = pt[1]
            cone.colour = 0
            cone_map_test.cones.append(cone)

        for pt in right_arr:
            cone = Cone()
            cone.pose.pose.position.x = pt[0]
            cone.pose.pose.position.y = pt[1]
            cone.colour = 1
            cone_map_test.cones.append(cone)

        return cone_map_test

    def get_map_boundary(self, cone_map:list):
        #Get maximum and minimum x and y
        max_x = float('-inf')
        min_x = float('inf')
        max_y = float('-inf')
        min_y = float('inf')
        
        for cone in cone_map:
            x, y, theta, covariance, _ = self.extract_cone_data(cone)
            if x > max_x:
                max_x = x
            elif x < min_x:
                min_x = x
            
            if y > max_y:
                max_y = y
            elif y < min_y:
                min_y = y

        return max_x, min_x, max_y, min_y

    def extract_cone_data(self, cone_input : Cone) -> (float, float, float, list[float], int):
        x = cone_input.pose.pose.position.x
        y = cone_input.pose.pose.position.y
        theta = cone_input.pose.pose.orientation.w
        covaraince = cone_input.pose.covariance
        colour = cone_input.colour
        
        return x, y, theta, covaraince, colour

    def plot_occupancy_grid(self, x_list, y_list, occupancy_grid_matrix):
        #Transpose it for having column to x and row to y:
        occupancy_grid_matrix = np.transpose(occupancy_grid_matrix)

        #Grid plotting
        X,Y = np.meshgrid(x_list, y_list)
        Z = np.sin(np.sqrt(X ** 2 + Y ** 2))
        print(Z.shape)
        print(occupancy_grid_matrix.shape)

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

    def interpolate_bounds(self, occ_grid : np.ndarray, left : np.ndarray, right : np.ndarray, x_list : np.ndarray, y_list : np.ndarray):
        '''
            Interpolates points between cones to complete the track boundary
            
            Inputs: 
            occ_grid = Occupancy grid with cones inputted
            left = list cone coordinates for left boundary
            right = list cone coordinates for right boundary

            Output:
            
        '''
        
        x_l, y_l = left[::,0], left[::,1]
        x_r, y_r = right[::,0], right[::,1] 
        
        mat_h, mat_w = np.shape(occ_grid)
        
        spline_tck_l, u_l = splprep([x_l, y_l], s=2, per=True) # can tune s
        spline_tck_r, u_r = splprep([x_r, y_r], s=2, per=True) # can tune s

        eval_points = 10**(math.ceil(math.log10(mat_w))+1)

        # evaluate spline at given point #TODO fix 
        xi_l, yi_l = splev(np.linspace(0,1,eval_points), spline_tck_l) 
        xi_r, yi_r = splev(np.linspace(0,1,eval_points), spline_tck_r)

        interp_left, interp_right = np.column_stack((xi_l, yi_l)), np.column_stack((xi_r, yi_r))

        for pt in np.vstack((interp_left, interp_right)):
            x = round(pt[0])
            y = round(pt[1])
            occ_grid[y][x] = 255

        return 

    def bodyfit_adjust(self, occ_grid : np.ndarray, width : float) -> np.ndarray:
        '''
            Fills and shrinks the track boundary by half the width of the car on each side (left/right) 
            of the track    
        
            Inputs: 
            occ_grid = Occupancy grid with interpolated boundary
            width = width of car in metres

            Output:
            - shrunken_track = Occupancy grid that depicts drivable area and accounts 
            for width of the car
        '''
    
        start_x, start_y, theta, covaraince, colour = self.extract_cone_data(self.start_pos)
        cv.imwrite('prefill.png', occ_grid)
        cv.floodFill(occ_grid, None, (int(start_x), int(start_y)), 255)
        cv.imwrite('filled.png', occ_grid)

        shrinkage_amount = width / self.resolution
        kernel_size = (math.ceil(shrinkage_amount / 2), math.ceil(shrinkage_amount / 2))
        kernel = np.ones(kernel_size, np.uint8)

        shrunken_track = cv.erode(occ_grid, kernel)
        
        return shrunken_track

    def sep_l_r_bounds(self, occ_grid : np.ndarray) -> (np.ndarray, np.ndarray):
        '''
            Extracts the left and right edges of the track from the occupancy grid 
            
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
            if cv.contourArea(contours[i]) < 100:
                continue
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

        cv.imshow('contours', new)
        cv.imwrite('contours.png', new)

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