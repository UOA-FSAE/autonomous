cmake_minimum_required(VERSION 3.8)
project(cone_detection_cpp_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})

# find dependencies
add_executable(cone_detection src/cone_detection.cpp)
ament_target_dependencies(cone_detection rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  cone_detection
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()