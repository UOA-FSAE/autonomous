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
#include "fsae_interfaces/msg/detections.hpp"

using std::placeholders::_1;

class ConeSubscriber : public rclcpp::Node
{
public:
  ConeSubscriber()
  : Node("cone_subscriber")
  {
    subscription_ = this->create_subscription<fsae_interfaces::msg::Detections>(
      "cone_detection", 10, std::bind(&ConeSubscriber::cone_detection_callback, this, _1));
  }

private:
  void cone_detection_callback(const fsae_interfaces::msg::Detections & msg) const
  {
    for (auto i = 0u; i < msg.blue.size(); i++)
    {
      float distance = sqrt(pow(msg.blue[i].x, 2) + pow(msg.blue[i].y, 2));
      std::cout << "blue: " << i << " " << msg.blue[i].x << " " << msg.blue[i].y << " " << distance << std::endl;
    }
  }
  rclcpp::Subscription<fsae_interfaces::msg::Detections>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
