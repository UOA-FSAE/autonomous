#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from moa_msgs.msg import ConeMap

import os
import sys
import matplotlib.pyplot as plt
import numpy as np
from ultralytics import YOLO 
import math 
from scipy.spatial.transform import Rotation as R

## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join('Formula-Student-Driverless-Simulator', 'python'))
sys.path.insert(0, path)
# sys.path.append('/home/Formula-Student-Driverless-Simulator/python')
# print(sys.path)
import fsds

class get_cones(Node):
    def __init__(self):
        super().__init__("get_cones")

        self.plot = False
        # connect to the simulator 
        self.client = fsds.FSDSClient(ip=os.environ['WSL_HOST_IP'])
        # Check network connection, exit if not connected
        self.client.confirmConnection()
        # After enabling setting trajectory setpoints via the api. 
        self.client.enableApiControl(True)

        # Load Engine
        self.tensorrt_model = YOLO("src/driverless_sim/simulator/resource/cone_detection.engine")

        # Set Camera parameters
        self.HFoV = math.radians(120)  # Horizontal field of view in degrees, converted to radians

        # create publisher
        self.sim_cone_pub = self.create_publisher(ConeMap, "cone_map", 10)

        self.create_timer(1.0, self.get_cones_from_simulator)


    def get_cones_from_simulator(self):
        # Get the image
        [image] = self.client.simGetImages([fsds.ImageRequest(camera_name = 'zed_camera', image_type = fsds.ImageType.Scene, pixels_as_float = False, compress = True)], vehicle_name = 'FSCar')
        fsds.write_file(os.path.normpath('temp.png'), image.image_data_uint8)

        # Calculate other Camera Parameters
        image_width = image.width  # Image width in pixels
        image_height = image.height  # Image height in pixels
        VFoV = self.HFoV * (image_height / image_width)  # Calculate vertical FoV

        # Run inference
        results = self.tensorrt_model('temp.png')
        boxes = results[0].boxes.xyxy.tolist()
        classes = results[0].boxes.cls.tolist()
        names = results[0].names
        confidences = results[0].boxes.conf.tolist()

        # Get car orientation
        imu = self.client.getImuData(imu_name = 'Imu', vehicle_name = 'FSCar')
        car_orientation = (imu.orientation.x_val,imu.orientation.y_val,imu.orientation.z_val,imu.orientation.w_val)

        # Get car position
        car_pos = self.get_car_position()

        lb = []
        rb = []

        # Iterate through the results
        for box, cls, conf in zip(boxes, classes, confidences):
            x1, y1, x2, y2 = map(int, map(round, box))
            confidence = conf
            detected_class = cls
            name = names[int(cls)]

            if detected_class==1.0:
                cone_height_real = 505/1000
            else:
                cone_height_real = 325/1000

            # Detected cone in the image
            u = int((x1+x2)/2)  # X-coordinate of cone center in pixels
            v = int((y1+y2)/2)   # Y-coordinate of cone center in pixels
            cone_height_image = y2-y1  # Detected height of cone in image (in pixels)

            # Compute depth (Z), which is also the x direction, according to car's coordinate system
            Z = (cone_height_real * image_height) / (2 * cone_height_image * math.tan(VFoV / 2)*0.9)

            # Compute X and Y coordinates relative to the camera
            X = Z * (u / image_width - 0.5) * 2.75 * math.tan(self.HFoV / 2) # The horizontal difference
            Y = Z * (v / image_height - 0.5) * 2.75 * math.tan(VFoV / 2) # The vertical difference

            global_X, global_Y, global_Z = self.transform_cone_to_global(Z, X, Y, car_pos, car_orientation)

            if detected_class == 0.0:  # type 0 color cone is blue
                lb.append([global_X,global_Y,0.0])
            elif detected_class == 4.0:    # type 4 color cone is yellow
                rb.append([global_X,global_Y,0.0])

        msg = self.get_cone_map_msg(lb.copy(),rb.copy())   # convert the boundaries to point list

        self.sim_cone_pub.publish(msg)  # publish

    def get_car_position(self):
        """returns current position of the simulator car"""
        position = self.client.getCarState().kinematics_estimated.position  # car kinetmatics in ENU coordinates
        
        return position.x_val, position.y_val, 0.0


    def get_cone_map_msg(self, lb, rb):
        try:
            assert len(lb) == len(rb)
        except Exception as e:
            self.get_logger().error(e)
        finally:
            for i in range(len(lb)):
                lb[i] = Point(x=lb[i][0],y=lb[i][1],z=lb[i][2])
                rb[i] = Point(x=rb[i][0],y=rb[i][1],z=rb[i][2])

        return ConeMap(left_cones=lb, right_cones=rb)


    def transform_cone_to_global(self, X, Y, Z, car_pos, car_orientation):
        """
        Transforms the cone's local coordinates to global coordinates.
        
        Parameters:
            X, Y, Z: Cone coordinates in the camera's local frame.
            car_pos: Tuple (car_x, car_y, car_z) representing the car's global position.
            car_orientation: Tuple (orint_x, orint_y, orint_z, orint_w) representing the car's orientation as a quaternion.
            
        Returns:
            Global coordinates of the cone (global_X, global_Y, global_Z).
        """
        # Convert car's orientation (quaternion) to a rotation matrix
        r = R.from_quat([car_orientation[0], car_orientation[1], car_orientation[2], car_orientation[3]])
        rotation_matrix = r.as_matrix()
        
        # Cone's local position in camera frame
        cone_local_pos = np.array([X, Y, Z])
        
        # Rotate the local coordinates to the global frame
        cone_global_pos = rotation_matrix @ cone_local_pos
        
        # Translate by the car's global position
        global_X = car_pos[0] + cone_global_pos[0]
        global_Y = car_pos[1] + cone_global_pos[1]
        global_Z = car_pos[2] + cone_global_pos[2]
        
        return global_X, global_Y, global_Z


def main(args=None):
    rclpy.init(args=args)

    simulator_cones = get_cones()

    rclpy.spin(simulator_cones)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator_cones.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()