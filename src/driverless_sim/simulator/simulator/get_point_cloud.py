#!/usr/bin/python3

import os
import sys
import matplotlib.pyplot as plt

## adds the fsds package located the parent directory to the pyhthon path
path = os.path.abspath(os.path.join('Formula-Student-Driverless-Simulator', 'python'))
sys.path.insert(0, path)
# sys.path.append('/home/Formula-Student-Driverless-Simulator/python')
# print(sys.path)
import fsds

# connect to the AirSim simulator 
client = fsds.FSDSClient()

# Check network connection
client.confirmConnection()

lidardata = client.getLidarData(lidar_name = 'Lidar1')

# nanosecond timestamp of when the imu frame was captured
print("lidardata nano: ", lidardata.time_stamp)

# the location of the lidar at the moment of capture in global reference frame
print("lidar pose: ", lidardata.pose)

# Convert the list of floats into a list of xyz coordinates
points = numpy.array(lidardata.point_cloud, dtype=numpy.dtype('f4'))
points = numpy.reshape(points, (int(points.shape[0]/3), 3))

print("number of hit points: ", len(points))

x_values = [point[0] for point in points]
y_values = [point[1] for point in points]
z_values = [point[2] for point in points]

# Plotting the 3D scatter plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
ax.scatter(x_values, y_values, z_values, c='b', marker='o')

# Set labels and title
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
plt.title('3D Scatter Plot of Points')

# Display the plot
plt.show()