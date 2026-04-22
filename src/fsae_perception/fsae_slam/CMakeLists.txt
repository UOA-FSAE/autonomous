cmake_minimum_required(VERSION 3.8)
project(fsae_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fsae_interfaces REQUIRED)

# ---------- cone_landmark_mapper_py node (Python — active implementation) ----------
install(PROGRAMS
  reference_cone_landmark_mapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cone_landmark_mapper_py
)
 
# ---------- cone_landmark_mapper node (C++ stub — kept for future porting) ----------
add_executable(cone_landmark_mapper src/cone_landmark_mapper.cpp)
target_include_directories(cone_landmark_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cone_landmark_mapper PUBLIC cxx_std_17)
ament_target_dependencies(cone_landmark_mapper
  rclcpp std_msgs geometry_msgs fsae_interfaces)

install(TARGETS cone_landmark_mapper
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
