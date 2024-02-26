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

from std_msgs.msg import Float32, Float64
from ackermann_msgs.msg import AckermannDrive


class ackermann_to_steer(Node):

    def __init__(self):
        super().__init__('ackermann_to_steer_torque_angle')

        # Declare a parameter with a default value
        self.declare_parameter('car_name', 'test')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value

        self.Integral_speed = 0
        self.Integral_angle = 0

        self.speed_ref = 0
        self.steering_angle_ref = 0

        # Publishing topics
        steering_topic = "/" + car_name + "/cmd_steering"
        torque_topic = "/" + car_name + "/cmd_throttle"
        speed_topic = "/" + car_name + "/speed"
        print(steering_topic)

        self.steering_publisher = self.create_publisher(Float32, steering_topic, 10)
        self.torque_publisher = self.create_publisher(Float32, torque_topic, 10)

        # Subscribing topics
        self.ackermann_subscribe = self.create_subscription(
            AckermannDrive,
            '/drive',
            self.set_reference,
            10)
        self.ackermann_subscribe  # prevent unused variable warning

        self.feedback_subscribe = self.create_subscription(
            Float64,
            speed_topic,
            self.PI_controller,
            10)
        self.feedback_subscribe  # prevent unused variable warning

        # Tune the following parameter for PI controller
        # Only turn on I if P term cant make turn angle and speed converge
        self.P_speed = 1000
        self.I_speed = 0

        # Tune these two only if need
        self.abs_integral_speed_max = 400
        self.abs_integral_angle_max = 20.000

        print("Initialized")

    def PI_controller(self, msg):
        current_speed = msg.data

        error_speed = self.speed_ref - current_speed

        if abs(self.Integral_speed) <= self.abs_integral_speed_max:
            self.Integral_speed += current_speed

        output_torque = error_speed * self.P_speed + self.Integral_speed * self.I_speed

        msg_torque = Float32()

        msg_torque.data = output_torque
        #print(current_speed)
        self.torque_publisher.publish(msg_torque)

    def set_reference(self, msg):
        self.speed_ref = msg.speed
        target_steer = msg.steering_angle

        msg_steer = Float32()
        if target_steer > self.abs_integral_angle_max:
            target_steer = float(self.abs_integral_angle_max)
        elif target_steer < -1 * self.abs_integral_angle_max:
            target_steer = float(-1 * self.abs_integral_angle_max)

        msg_steer.data = target_steer
        print(target_steer)
        self.steering_publisher.publish(msg_steer)

def main(args=None):
    rclpy.init(args=args)

    ackermann_to_steer_node = ackermann_to_steer()

    rclpy.spin(ackermann_to_steer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ackermann_to_steer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
