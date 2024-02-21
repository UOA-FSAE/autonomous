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
from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovariance;
from geometry_msgs.msg import Pose;

import math
import numpy as np
from scipy.stats import norm

class Virtual_Cone_Detection(Node):
    stored_data_string_list = ""

    def pack_to_conemap(self, string_input):
        output_message = self.convert_message_to_data(string_input)
        return output_message;

    def __init__(self):
        super().__init__('virtual_cone_detection')
        self.publisher_ = self.create_publisher(ConeMap, 'cone_map', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        with open('/ws/src/perception/virtual_sensor/TESTDATA.txt') as f:
        	self.stored_data_list = f.readlines()
        	self.total_length = len(self.stored_data_list)
        	#print(self.stored_data_list)

    def timer_callback(self):
        string_input = self.stored_data_list[self.i];
        msg = self.pack_to_conemap(string_input)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        if (self.i + 1) == self.total_length:
        	self.i = 0;
        else:
        	self.i += 1

    def convert_message_to_data(self, data: str):
        #Convert message into ConeMap message
        #The 0th cone in ConeMap is the cart's current location and orientation where position is the position of the cart and orientation's w is the orientation of the cart, the rest of the messages are the cones mapped locally
        output_message = ConeMap();
        cones_list = [];
        
        list_of_messages = data.split(",");
        x = float(list_of_messages[1]);
        y = float(list_of_messages[2]);
        theta = float(list_of_messages[3]);
        cart = self.pack_cone_message(x,y,theta,0)
        cones_list.append(cart);

        #Create cone object for recording cart position and rotation status
        list_of_cone_messages = list_of_messages[4::1];
        print(list_of_cone_messages);
        #list_of_local_cones_x = [];
        #list_of_local_cones_y = [];
        
        for index in range(0,len(list_of_cone_messages),1):
            if index % 3 == 0:
                if list_of_cone_messages[index] != "\n":
                    individual_x = float(list_of_cone_messages[index].strip()[1::1]);
                    individual_y = float(list_of_cone_messages[index + 1].strip()[0:len(list_of_cone_messages[index + 2].strip()):1]);
                    individual_color = float(list_of_cone_messages[index + 2].strip()[0:len(list_of_cone_messages[index + 2].strip()) - 1:1]);
                    #list_of_local_cones_x.append(float(individual_x));
                    #list_of_local_cones_y.append(float(individual_y));
                    individual_cone = self.pack_cone_message(individual_x,individual_y,0.00, individual_color)
                    cones_list.append(individual_cone); #Add to cone messaage

        #list_of_cones = np.array([list_of_local_cones_x, list_of_local_cones_y])

        output_message.cones = cones_list;
                
        #return x, y, theta, list_of_cones;
        return output_message

    def pack_cone_message(self,x,y,theta, color):
        output_cone = Cone();
        position = Point();
        orientation = Quaternion();
        pose_with_covariance = PoseWithCovariance();
        pose = Pose();
        position.x = self.contaminate_message(x, 1e-6);
        position.y = self.contaminate_message(y, 1e-6);
        orientation.w = self.contaminate_message(theta, 1e-6);
        pose.position = position;
        pose.orientation = orientation;
        pose_with_covariance.pose = pose;
        output_cone.pose = pose_with_covariance;
        output_cone.colour = int(color);
        return output_cone

    def contaminate_message(self, data_input, noise_scale):
        #Create noisy signal from the data input
        output_number = np.random.normal(loc=data_input, scale = noise_scale, size=1)
        return float(output_number[0])
        

def main(args=None):
    
    rclpy.init(args=args)

    cone_detection = Virtual_Cone_Detection()

    rclpy.spin(cone_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
