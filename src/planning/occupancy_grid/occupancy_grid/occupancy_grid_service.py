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
        pass;

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