
'''
A ros bag generator for generation of desired cone map and the raw sensor data.
Useful resources: https://www.bilibili.com/video/BV1FW4y1M7PV/?spm_id_from=333.337.search-card.all.click
'''

from colorsys import yiq_to_rgb
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
import math
from datetime import date

class car:
    # A virtual test bot on python that runs through the world to get dataset
    # The virtual bot can't do translation and rotation at the same time. It must stop and rotate. 
    # The x,y,theta is measured in global reference frame where theta is the angle of the orientation of the cart from the x direction.
    pos_x = 0;
    pos_y = 0;
    theta = 0;
    vel_x = 0;
    vel_y = 0;
    theta_dot = 0;
    speed_unit = 0;
    angular_speed_unit = 0;
    t = 0;
    sensing_range = 0;
    recorded_data = [];

    def __init__(self, x, y, theta, speed_unit, angular_speed_unit, sensing_range):
        self.pos_x = x;
        self.pos_y = y;
        self.theta = theta;
        self.vel_x = 0;
        self.vel_y = 0;
        self.theta_dot = 0;
        self.speed_unit = speed_unit;
        self.angular_speed_unit = angular_speed_unit;
        self.t = 0;
        self.recorded_data = [];
        self.sensing_range = sensing_range

    #State getter
    def return_position(self):
        return np.array([self.pos_x, self.pos_y, self.theta])

    def return_velocity(self):
        return np.array([self.vel_x, self.vel_y]);

    #Motion
    def time_lapsing(self, delta_t):  #Main function for updating the state of the cart and receiving sensor data
        #Actuate
        self.pos_x = self.vel_x * delta_t + self.pos_x;
        self.pos_y = self.vel_y * delta_t + self.pos_y;
        self.theta = self.theta_dot * delta_t + self.theta;
        self.t = delta_t + self.t;
        #print("State at t =", self.t, ":", self.pos_x, "unit", ",", self.pos_y, "unit", ",", self.theta * 180 / np.pi, "degree")

    def change_rotation(self, speed):  #Change rotation speed of the cart
        self.stop();
        self.theta_dot = speed * self.angular_speed_unit;

    def change_translation(self, speed): #Change linear speed of the cart
        self.stop();
        self.vel_x = speed * self.speed_unit * np.cos(self.theta);
        self.vel_y = speed * self.speed_unit * np.sin(self.theta);

    def stop(self):
        self.theta_dot = 0;
        self.vel_x = 0;
        self.vel_y = 0;

    def detect(self, cones_in_global_frame):
        rotation_angle = self.theta - np.pi/2;
        rotation_matrx = np.array([[np.cos(rotation_angle), np.sin(rotation_angle)],[-np.sin(rotation_angle), np.cos(rotation_angle)]]);
        cones_in_car_frame_no_rotation = cones_in_global_frame - np.array([[self.pos_x],[self.pos_y]]);
        cones_in_car_frame = np.matmul(rotation_matrx, cones_in_car_frame_no_rotation)
        measured_cones_x = [];
        measured_cones_y = [];
        for i in range(0,cones_in_car_frame[0].size,1):
            if (cones_in_car_frame[1][i] > 0) and (math.sqrt(cones_in_car_frame[0][i] ** 2 + cones_in_car_frame[1][i] ** 2) < self.sensing_range):
                measured_cones_x.append(cones_in_car_frame[0][i]);
                measured_cones_y.append(cones_in_car_frame[1][i])
        self.recorded_data.append([self.t, measured_cones_x, measured_cones_y, self.return_position()[0], self.return_position()[1], self.return_position()[2]])
        
        return cones_in_car_frame

    def return_recorded_data(self):
        return self.recorded_data;


def cone_arrangement_generation(number_of_cones, left_boundary, right_boundary, top_boundary, bottom_boundary):
    x_list = np.random.uniform(left_boundary, right_boundary, size=number_of_cones)
    y_list = np.random.uniform(bottom_boundary, top_boundary, size=number_of_cones)
    cone_list = np.array([x_list,y_list]);
    return cone_list, x_list, y_list;

def time_lapsing(graph_arrow, point_detected_cones, car, time, animation_scale, cones_in_global):
    for i in range(0, time * 100, 1):
        car.time_lapsing(0.01);
        new_x = CAR.return_position()[0]
        new_y = CAR.return_position()[1]
        new_theta = CAR.return_position()[2]
        new_dx = 1 * np.cos(new_theta);
        new_dy = 1 * np.sin(new_theta);
        graph_arrow.set_data(x = new_x, y = new_y, dx = new_dx, dy = new_dy)

        #Change graphs for cones under car's reference frame
        detected_cone = car.detect(cones_in_global)
        point_detected_cones = ax2.plot(detected_cone[0], detected_cone[1], marker="o", linestyle = 'None')
        
        plt.draw()
        plt.pause(0.01 / animation_scale) 

def DCM(theta):
    return np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]]);
#
# Main code below:
#

# Set file name for saving rosbag data
file_name = "test-data-5-cones.txt"
test_data_output_string = ""

# Clearing a file
decimal_place = 4;

head_for_cone_list = "List of cones" + "\n" + "x, y\n"
head_for_sensor_data = "List of real time data" + "\n" + "t, x, y, theta, (distance(unit), theta(rad)) lists\n"

# Initialize world and car 
CAR = car(5,0,0,1,np.pi/180,1000);  #It is setted such that for each second, the rotation is one degree, and the translation is one unit
left_boundary = 0;
right_boundary = 100;
top_boundary = 100;
bottom_boundary = 0;
number_of_cones  = 5;
cone_list, real_cone_x, real_cone_y = cone_arrangement_generation(number_of_cones, left_boundary, right_boundary, top_boundary, bottom_boundary);
real_cone_x = [76.5979, 96.1476, 92.0906, 62.8596, 26.6301];
real_cone_y = [76.2157, 90.4749, 16.2427, 74.0812, 17.7761];
cone_list = np.array([real_cone_x, real_cone_y])

# Write existing cone into the file
test_data_output_string = test_data_output_string + head_for_cone_list
for index in range(0,len(real_cone_x),1):
    append_string = str(round(real_cone_x[index],decimal_place)) + "," + str(round(real_cone_y[index],decimal_place)) + "\n";
    test_data_output_string = test_data_output_string + append_string

animation_scale = 1;     #animation scale: the animation speed is animation_scale x real time
# Define landmarks

# create figure and axis
fig, ax = plt.subplots()
fig2, ax2 = plt.subplots()

# set axis limits
ax.set_xlim([left_boundary, right_boundary])
ax.set_ylim([bottom_boundary, top_boundary])
ax.axis('equal')
ax.set_title("Car motion in the world")
ax.set_xlabel('x (unit)')
ax.set_ylabel('y (unit)')

ax2.set_xlim([-100, 100])
ax2.set_ylim([-100, 100])
ax2.axis('equal')
ax2.set_title("Sensed cone data in space")
ax2.set_xlabel('sideway direction (unit)')
ax2.set_ylabel('frontal direction (unit)')

cart_symbol = ax.arrow(0, 0, 1, 0, head_width=1, head_length=1, fc='k', ec='k')
ax.scatter(cone_list[0], cone_list[1], marker="x")
initial_detected_cone = CAR.detect(cone_list);
point_detected_cones = ax2.plot(initial_detected_cone[0], initial_detected_cone[1], marker="X", linestyle = 'None')

# List of explicit command provided by t and command
# Car instructions
##
#CAR.change_translation(10);
#time_lapsing(cart_symbol, point_detected_cones, CAR, 1, animation_scale, cone_list);
CAR.change_rotation(45);
time_lapsing(cart_symbol, point_detected_cones, CAR, 2, animation_scale, cone_list);
#CAR.change_translation(10);
#time_lapsing(cart_symbol, point_detected_cones, CAR, 10, animation_scale, cone_list);
##
##
##

# Record all sensor data in format
test_data_output_string = test_data_output_string + head_for_sensor_data

sensor_data = CAR.return_recorded_data();
for index in range(0, len(sensor_data), 1):
    append_string = str(round(sensor_data[index][0], decimal_place)) +  ","  + str(round(sensor_data[index][3], decimal_place)) +  ","  + str(round(sensor_data[index][4], decimal_place)) + "," + str(round(sensor_data[index][5], decimal_place)) + ","  
    for sensor_index in range(0, len(sensor_data[index][1]), 1):
        append_string = append_string + "(" + str(round(sensor_data[index][1][sensor_index], decimal_place)) + "," + str(round(sensor_data[index][2][sensor_index], decimal_place)) + ")"
        if not(sensor_index == (len(sensor_data[index][1]) - 1)):
            append_string = append_string + ","
    append_string = append_string + "\n"
    test_data_output_string = test_data_output_string + append_string
    append_string = ""

test_data_output_string = test_data_output_string + "end\n"
print(test_data_output_string)

# Write output string to file
text_file_output = open(file_name, mode = "w", buffering = -1);
text_file_output.write(test_data_output_string);
text_file_output.close()

# show plot
plt.show()
