
# Compare Two methods with the large orange cones' position.

import sys
import os
from ultralytics import YOLO 
import math 
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join('Formula-Student-Driverless-Simulator', 'python'))
sys.path.insert(0, path)

import fsds
import numpy as np

# connect to the simulator 
client = fsds.FSDSClient()

# Check network connection, exit if not connected
client.confirmConnection()

#Load Engine
tensorrt_model = YOLO("src/driverless_sim/simulator/resource/cone_detection.engine")


# Get the image
[image] = client.simGetImages([fsds.ImageRequest(camera_name = 'zed_camera', image_type = fsds.ImageType.Scene, pixels_as_float = False, compress = True)], vehicle_name = 'FSCar')
fsds.write_file(os.path.normpath('temp.png'), image.image_data_uint8)

# Camera parameters
HFoV = math.radians(120)  # Horizontal field of view in degrees, converted to radians
image_width = image.width  # Image width in pixels
image_height = image.height  # Image height in pixels
VFoV = HFoV * (image_height / image_width)  # Calculate vertical FoV

# Run inference
results = tensorrt_model('temp.png')

boxes = results[0].boxes.xyxy.tolist()
classes = results[0].boxes.cls.tolist()
names = results[0].names
confidences = results[0].boxes.conf.tolist()

cone_list = []

imu = client.getImuData(imu_name = 'Imu', vehicle_name = 'FSCar')
orint_x = imu.orientation.x_val
orint_y = imu.orientation.y_val
orint_z = imu.orientation.z_val
orint_w = imu.orientation.w_val

def get_car_position():
        """returns current position of the simulator car"""
        position = client.getCarState().kinematics_estimated.position  # car kinetmatics in ENU coordinates
        
        return position.x_val, position.y_val, 0.0

(car_x,car_y,car_z) = get_car_position()

car_pos = (car_x, car_y, car_z)  # Car's global position
car_orientation = (orint_x, orint_y, orint_z, orint_w)  # Car's orientation as quaternion

print(car_pos)
print(car_orientation)

rotation = R.from_quat(car_orientation)
# Convert to Euler angles (yaw, pitch, roll) in radians
euler_angles = rotation.as_euler('xyz', degrees=True)
roll, pitch, yaw = euler_angles
print(f"Yaw: {yaw:.2f}, Pitch: {pitch:.2f}, Roll: {roll:.2f}")

def transform_cone_to_global(X, Y, Z, car_pos, car_orientation):
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

    print(cone_global_pos)
    
    # Translate by the car's global position
    global_X = car_pos[0] + cone_global_pos[0]
    global_Y = car_pos[1] + cone_global_pos[1]
    global_Z = car_pos[2] + cone_global_pos[2]
    
    return global_X, global_Y, global_Z

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

    # Compute depth (Z)
    Z = (cone_height_real * image_height) / (2 * cone_height_image * math.tan(VFoV / 2)*0.9)

    # Compute X and Y coordinates relative to the camera
    X = Z * (u / image_width - 0.5) * 2.75 * math.tan(HFoV / 2)
    Y = Z * (v / image_height - 0.5) * 2.75 * math.tan(VFoV / 2)

    print(name)
    print(detected_class)

    if detected_class==1.0:
        global_X, global_Y, global_Z = transform_cone_to_global(Z, X, Y, car_pos, car_orientation)
        print(name)
        cone_list.append([global_X,global_Y])
        print(f"3D position of the cone relative to the camera: X = {X:.2f}m, Y = {Y:.2f}m, Z = {Z:.2f}m")

    #Horizontal Difference:X
    #Depth Difference :Z

    # Print the 3D position of the cone relative to the camera


def get_vector3r(vec, scale):
    """Scales the given vector using the given real scalar"""
    return [vec[0]*scale, -vec[1]*scale, vec[2]*scale]

def get_local_ENU(position, world_to_meters=100):
        """Transform Unreal engine coordinates (UU) to AirSim coordiantes (ENU) - currently there is translation and scale (cm to m) I think not 100% sure
        For more details please see the below documents 
        1. https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/coordinate-frames/#unreal-engine
        2. https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/UE4Project/Plugins/AirSim/Source/CoordFrameTransformer.cpp
        """
        local_offset = np.array([4575.15,8577.82,0])    # translation vector from UU to ENU (fixed for now)
        return get_vector3r(position - local_offset, 1/world_to_meters)

def find_cones():
        """ Detects cones in the simulator
        HOWEVER at the moment the exact positions of the cones is requested from API (change asap)"""
        # referee state
        ref_state = client.getRefereeState()

        cones = ref_state.cones  # list of cones where each cone is in a dictionary format e.g., [{cone1}, {cone2}, {cone3}]
        leftboundary = []
        rightboundary = []
        start_end = []

        for cone_dict in cones:
            color = cone_dict['color']
            x = cone_dict['x']
            y = cone_dict['y']
            z = 0.0 # z value is currently not provided 

            x,y,z = get_local_ENU([x,y,z]) # transform from UU to ENU coordinates 

            if color == 0:  # type 0 color cone is blue
                rightboundary.append([x,y,z])
            elif color == 1:    # type 1 color cone is yellow
                leftboundary.append([x,y,z])
            elif color == 2:    # orange start/end cones
                start_end.append([x,y,z])

        return leftboundary, rightboundary, start_end

lb, rb, start_end = find_cones()
print(start_end)
print(cone_list)

# Extract x and y values
y = [point[0] for point in start_end]
x = [point[1] for point in start_end]

# Create the plot
plt.figure(figsize=(8, 6))
plt.scatter(x, y, marker='o')  # Use 'o' to show points

y = [point[0] for point in cone_list]
x = [point[1] for point in cone_list]
plt.scatter(x, y, marker='o')  # Use 'o' to show points
plt.title('XY Plot')
plt.xlabel('X values')
plt.ylabel('Y values')
plt.grid()
plt.axis('equal')  # Equal scaling for both axes
plt.show()