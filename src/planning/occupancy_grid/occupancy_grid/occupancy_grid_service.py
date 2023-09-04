import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


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
        #Get cone map size
        #Initialize matrix with size of map
        resolution = 1; # In meters
        #Put high integer value on cone occupancy grid
        #Interpolate between cones to get full occupancy grid
        #Output
        pass;

    ##############################Boundary mapping
    def interpolate_grid(self, points):
        """
        Description:

        output: 
        Matrix of occupancy grid
        Resolution as float
        Left boundary
        -	pointArray
        Right boundary
        -	pointArray

        """
        num_points = len(points)
        resolution = 5

        # Calculate pairwise distances between points
        distances = np.linalg.norm(points[:, np.newaxis, :] - points[np.newaxis, :, :], axis=-1)

        print(distances)

        # Create a graph with distances as weights
        G = nx.Graph()
        for i in range(num_points):
            for j in range(i + 1, num_points):
                G.add_edge(i, j, weight=distances[i, j])

        # Approximate the TSP using a minimal spanning tree
        tsp_tree = nx.minimum_spanning_tree(G)
        tsp_path = list(nx.dfs_preorder_nodes(tsp_tree, source=0))

        #======================== 
        #convery tsp_path to control_points
        control_points = np.array(tsp_path)

        #========================
        # Fit a spline to the control points
        tck, u = splprep([control_points[:, 0], control_points[:, 1]], s=0)

        # Generate a finer parameter grid for interpolation
        u_fine = np.linspace(0, 1, len(control_points)*resolution)

        # Evaluate the spline at the parameter values to get interpolated points
        interpolated_path = np.column_stack(splev(u_fine, tck))

        # Plot the original control points
        plt.plot(control_points[:, 0], control_points[:, 1], 'ro', label='Control Points')

        # Plot the interpolated points along the winding loop
        plt.plot(interpolated_path[:, 0], interpolated_path[:, 1], 'b-', label='Interpolated Points')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Interpolation of Points along a Winding Loop')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()


        # =======================================
        # Define bitmap dimensions
        width, height = 400, 400
        occupancy_grid = np.ones((height, width, 3), dtype=np.uint8) * 255  # Initialize with white background

        # Mark intersecting pixels
        for point in interpolated_path:
            x, y = points[int(point)]
            x_pixel = int(x * (width - 1))
            y_pixel = int(y * (height - 1))
            occupancy_grid[y_pixel, x_pixel, :] = [0, 0, 0]  # Mark with black color

        # Display the bitmap
        plt.imshow(bitmap)
        plt.axis('off')
        plt.title('Bitmap of Intersecting Path')
        plt.show()

        return occupancy_grid, left_boundary, right_boundary, resolution



    ##############################Bodyfit adjustment


    ##############################Driveable region decision



def main():
    rclpy.init()

    minimal_service = Occupancy_grid()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()