cmake_minimum_required(VERSION 3.8)
project(cone_detection_cpp)

# Assuming 'cone_detection_model.engine' is in the 'resources' directory of your project
set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")
set(DEST_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Copy file from source directory to destination directory
add_custom_command(
    OUTPUT "${DEST_DIR}/cone_detection_model.engine"
    COMMAND ${CMAKE_COMMAND} -E copy "${SOURCE_DIR}/cone_detection_model.engine" "${DEST_DIR}/cone_detection_model.engine"
    DEPENDS "${SOURCE_DIR}/cone_detection_model.engine"
)

# Ensure the file is copied before the target is built
add_custom_target(copy_cone_detection_model ALL
    DEPENDS "${DEST_DIR}/cone_detection_model.engine"
)

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