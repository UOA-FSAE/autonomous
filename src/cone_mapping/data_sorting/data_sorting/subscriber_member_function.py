# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mapping_interfaces.msg import ConeMap
from mapping_interfaces.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance;
from geometry_msgs.msg import Pose;

#Plotting and mathematic related
import math
import numpy as np
import matplotlib.pyplot as plt 
import time

# Sort cone functions

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('data_sorter')
        print("started")
        self.subscription = self.create_subscription(
            ConeMap,
            'cone_detect',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cone_map = np.array([[],[]]);
        self.output_message = ConeMap(); #Message output that is ready to be logged.
        self.counter = 0
        self.stored_map = np.array([],[]);

    def listener_callback(self, msg):
        #self.get_logger().info('Mapped result: "%s"' % msg.cones)
        print("Listened")
        x, y, theta, list_of_cones = self.convert_message_to_data(msg)
        print(list_of_cones.transpose())
        #self.get_logger().info('Mapped result: "%s"' % self.cone_map)
        print(self.counter);
        if self.counter == 0:
            self.stored_map = list_of_cones.transpose();
        
        self.counter += 1;

    def convert_message_to_data(self, data):
        #Convert from Cone Map to data that is processed in the node
        list_of_cones = data.cones[1::1];
        cart_info = data.cones[0];
        
        x, y, theta = self.extract_data_from_cone(cart_info)

        list_of_local_cones_x = [];
        list_of_local_cones_y = [];
        
        for index in range(0,len(list_of_cones),1):
            individual_x, individual_y, individual_theta = self.extract_data_from_cone(list_of_cones[index])
            list_of_local_cones_x.append(individual_x);
            list_of_local_cones_y.append(individual_y);

        list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y])
                
        return x, y, theta, list_of_cones;

    def extract_data_from_cone(self, cone_input):
        x = cone_input.pose.pose.position.x;
        y = cone_input.pose.pose.position.y;
        theta = cone_input.pose.pose.orientation.w;
        return x, y, theta


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
