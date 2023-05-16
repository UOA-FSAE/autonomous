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

#This the main node that encode ackermann drive message to can message

import rclpy
from rclpy.node import Node
from zedmsg.msg import ObjectsStamped
from moa_msgs.msg import Cone
from moa_msgs.msg import ConeStamped

class detection(Node):

    def __init__(self):
        super().__init__('detector')
        
        self.publisher_ = self.create_publisher(ConeStamped, 'cone', 10)
        self.subscription = self.create_subscription(
            ObjectsStamped,
            'objects',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.msgSend = ConeStamped();
        

    def listener_callback(self, msg):
        
        for cones in msg.objects:
            msgSend.cone.confidence = cones.confidence
            msgSend.cone.colour = cones.label_id
            msgSend.cone.pose.pose.position.x = cones.position[1]
            msgSend.cone.pose.pose.position.y = cones.position[2]
            msgSend.cone.pose.pose.position.z = cones.position[3]
            msgSend.cone.pose.covariance = cones.position_covariance
            msgSend.cone.radius = cones.dimensions_3d[1];
            msgSend.cone.height = cones.dimensions_3d[2];
            msgSend.cone.id += 1
            self.publisher_.publish(msgSend)

def main(args=None):
    rclpy.init(args=args)

    detecter = detection()

    rclpy.spin(detecter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detecter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
