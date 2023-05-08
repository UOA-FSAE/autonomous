cmake_minimum_required(VERSION 3.5)
project(moa_msgs)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# List message and service files
set(msg_files
  "msg/CAN.msg"
  "msg/CANStamped.msg"
  "msg/Cone.msg"
  "msg/ConeStamped.msg"
  "msg/ConeMap.msg"
  "msg/ConeMapStamped.msg"
)
set(srv_files
  "srv/CANSendReq.srv"
)

# Generate message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
