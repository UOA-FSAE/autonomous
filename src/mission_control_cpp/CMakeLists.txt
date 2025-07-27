cmake_minimum_required(VERSION 3.8)
project(mission_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

include_directories(include)

# MissionControl executable
add_executable(mission_control src/mission_control.cpp)
ament_target_dependencies(mission_control rclcpp std_msgs rcl_interfaces)

# DummyNode executable
add_executable(dummy_node src/dummy_node.cpp)
ament_target_dependencies(dummy_node rclcpp std_msgs rcl_interfaces)

add_executable(dummy_publisher src/dummy_publisher.cpp)
ament_target_dependencies(dummy_publisher rclcpp std_msgs)

install(TARGETS
  mission_control
  dummy_node
  dummy_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
