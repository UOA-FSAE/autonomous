cmake_minimum_required(VERSION 3.5)
project(zed_wrapper)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
  set(ROS2_FOUND TRUE)
  #message("* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
  message("* ROS2 distro variable not set. Trying to figure it out...")
  set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;rolling")
  set(ROS2_FOUND FALSE)
  foreach(distro ${ROS2_DISTROS})
    if(NOT ROS2_FOUND)
      find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
      if(RCLCPP_H)
        #message("* Found ROS2 ${distro}")
        set(FOUND_ROS2_DISTRO ${distro})
        set(ROS2_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()

if(ROS2_FOUND)
  if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
    #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    add_definitions(-DFOUND_FOXY)
  elseif(${FOUND_ROS2_DISTRO} STREQUAL "iron")
    #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    add_definitions(-DFOUND_IRON)
  elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
    #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
    add_definitions(-DFOUND_HUMBLE)
  else()
    message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by the package '${PROJECT_NAME}'. Correct operation is not guaranteed.")
  endif()
else()
  message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#############################################
# Dependencies
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(zed_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

link_directories(
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GLEW_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${OpenGL_LIBRARY_DIRS}
)

set(ZED_LIBS
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OpenGL_LIBRARIES}
)

# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

###############################################################################
# SOURCES
set(ZED_WRAPPER_INC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cuda_utils.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/logging.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/yolo.hpp
)

set(ZED_WRAPPER_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/yolo.cpp
)

###############################################################################
# Bin and Install

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

add_executable(zed_wrapper src/zed_wrapper.cpp)

add_library(zed_wrapper_lib ${ZED_WRAPPER_SRC} ${ZED_WRAPPER_INC})

target_include_directories(zed_wrapper PUBLIC
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  zed_wrapper_lib
)

target_link_directories(zed_wrapper PUBLIC
  ${ZED_LIBS}
  zed_wrapper_lib
)

target_link_libraries(zed_wrapper
  ${ZED_LIBS}
  zed_wrapper_lib
)

ament_target_dependencies(zed_wrapper
  std_msgs
  rclcpp
  rclcpp_components
  zed_components
)

install(TARGETS zed_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install PARAMS files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
