cmake_minimum_required(VERSION 3.8)
project(track_utils)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the library
add_library(track_utils_lib
    src/curvature_calculator.cpp
)
target_include_directories(track_utils_lib PUBLIC include)
ament_target_dependencies(track_utils_lib geometry_msgs)

# Add the test executable
add_executable(test_curvature_calculator
    test/test_curvature_calculator.cpp
)
target_link_libraries(test_curvature_calculator track_utils_lib)
ament_target_dependencies(test_curvature_calculator geometry_msgs)

# Install library and executables
install(TARGETS
    track_utils_lib
    test_curvature_calculator
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
