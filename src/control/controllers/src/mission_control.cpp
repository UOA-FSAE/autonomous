#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "mission_control.hpp"


MissionController::MissionController(int argc, char ** argv) {
  // create a topic for other things to  

}


int main(int argc, char ** argv)
{
  rclcpp::init(arc, argv);
  missionControllerNode = MissionController();
  rclcpp.spin(missionControllerNode);

  printf("hello world controllers package\n");
  return 0;
}
