import numpy as np;
import math;
import matplotlib.pyplot as plt
import time;

def read_data(filename):
    input_file = open(filename, "r");
    sentences = input_file.readlines();
    input_file.close();
    index = 0
    #t, x, y, theta, (distance(unit), theta(rad)) lists
    t = [];
    x = [];
    y = [];
    theta = [];
    list_of_cone_input = [];
    for sentence in sentences:
        individual_sentence_list = sentence.split(",");
        individual_t = float(individual_sentence_list[0])
        individual_x = float(individual_sentence_list[1])
        individual_y = float(individual_sentence_list[2])
        individual_theta  = float(individual_sentence_list[3]);
        cone_list = individual_sentence_list[4::1];

        t.append(individual_t);
        x.append(individual_x);
        y.append(individual_y);
        theta.append(individual_theta)
        list_of_cone_input.append(get_cone_list(cone_list));
        index += 1;
    print(list_of_cone_input[0])

    return t,x,y,theta,list_of_cone_input
        
def get_cone_list(list_of_cone_messages):
    list_of_local_cones_x = [];
    list_of_local_cones_y = [];
        
    for index in range(0,len(list_of_cone_messages),1):
        if index % 2 == 0:
            individual_x = float(list_of_cone_messages[index].strip()[1::1]);
            individual_y = float(list_of_cone_messages[index + 1].strip()[0:len(list_of_cone_messages[index + 1].strip()) - 1:1])
            list_of_local_cones_x.append(float(individual_x));
            list_of_local_cones_y.append(float(individual_y));
    return np.array([list_of_local_cones_x, list_of_local_cones_y])

def convert_to_input_matrix(x: float, y: float, theta: float, list_of_cones):
    #Converting input message from cone detection into calculatable messages
    #t, x, y, theta, (distance(unit), theta(rad)) lists
    position_vector = np.array([[x],[y]]);
    rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]]) #Inverse DCM
    #No transformation for list of cones right now

    return position_vector, rotation_matrix, list_of_cones;

def create_cone_map(position_vector, rotation_matrix, list_of_cones):
    list_of_cones_unrotated = np.matmul(rotation_matrix, list_of_cones)
    list_of_cones_output = list_of_cones_unrotated + position_vector;
    return list_of_cones_output;

t,x,y,theta,list_of_cone_input = read_data("test_data.txt")
cone_map = np.array([[],[]])
for index in range(0, len(t), 1):
    individual_x = x[index];
    individual_y = y[index];
    individual_theta = theta[index];
    list_of_cones = list_of_cone_input[index];
    position_vector, rotation_matrix, list_of_cones = convert_to_input_matrix(individual_x, individual_y, individual_theta - np.pi/2, list_of_cones)
    new_cone_map = create_cone_map(position_vector, rotation_matrix, list_of_cones);
    cone_map = np.concatenate((cone_map, new_cone_map), axis=1)

#plt.scatter([76.5979, 96.1476, 92.0906, 62.8596, 26.6301],[76.2157, 90.4749, 16.2427, 74.0812, 17.7761])
plt.scatter(cone_map[0], cone_map[1])
plt.show()
