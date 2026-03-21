// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;

class LocalisationSubscriber : public rclcpp::Node
{
public:
  LocalisationSubscriber()
  : Node("localisation_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "car_position", 10, std::bind(&LocalisationSubscriber::car_position_callback, this, _1));
  }

private:
  void car_position_callback(const geometry_msgs::msg::Pose & msg) const
  {
    std::cout << "car_position : " << msg.position.x << " " << msg.position.y << " " << msg.position.z << " " << std::endl;
  }
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalisationSubscriber>());
  rclcpp::shutdown();
  return 0;
}
