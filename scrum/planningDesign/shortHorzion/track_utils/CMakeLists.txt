cmake_minimum_required(VERSION 3.8)
project(track_utils)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(corrade EXCLUDE_FROM_ALL) # so only things you use are built
add_subdirectory(magnum EXCLUDE_FROM_ALL)
add_subdirectory(test)

# Path where FindCorrade.cmake & FindMagnum.cmake can be found, adapt as needed
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "" ${CMAKE_PREFIX_PATH})
set(CMAKE_BUILD_RPATH "${PROJECT_SOURCE_DIR}/cmake_build_scrum")

find_package(Magnum REQUIRED ...) # see below

file(GLOB SRC_FILES  
    src/*.cpp
)

file(GLOB INCLUDE_FILES 
    src/*.hpp
)

# Add the library
add_library(track_utils_lib
    SRC_FILES
)

target_include_directories(track_utils_lib PUBLIC INCLUDE_FILES)
ament_target_dependencies(track_utils_lib geometry_msgs)


add_dependencies(track_utils_lib
    Magnum::Math)


# Add the test executable
add_executable(test_curvature_calculator
    test/test_curvature_calculator.cpp
)
target_link_libraries(test_curvature_calculator track_utils_lib)
# ament_target_dependencies(test_curvature_calculator geometry_msgs)

# Install library and executables
install(TARGETS
    track_utils_lib
    test_curvature_calculator
    DESTINATION lib/${PROJECT_NAME}
)

# ament_package()
