cmake_minimum_required(VERSION 3.5)
project(cone_detection_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

SET(SPECIAL_OS_LIBS "")

option(ENABLE_INT8_CALIBRATOR "Enable int8 calibrator class for quantification calibration, requires opencv dnn" OFF)

IF (ENABLE_INT8_CALIBRATOR)
    ADD_DEFINITIONS(-DENABLE_INT8_CALIBRATOR)
ENDIF()

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

#############################################
# Dependencies
find_package(ZED 4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moa_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# INCLUDES and LIBS
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

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

FILE(GLOB_RECURSE SRC_FILES src/cone_detection.c* src/yolo.c*)
FILE(GLOB_RECURSE HDR_FILES include/*.h*)

cuda_add_executable(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})
add_definitions(-O3 -D_MWAITXINTRIN_H_INCLUDED -Wno-deprecated-declarations)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

SET(TRT_LIBS nvinfer nvonnxparser)

target_link_libraries(${PROJECT_NAME}
                        ${TRT_LIBS}
                        ${SPECIAL_OS_LIBS}
                        ${ZED_LIBS}
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARIES}
                        ${GLEW_LIBRARIES}
                        ${OpenCV_LIBRARIES})

if(INSTALL_SAMPLES)
    LIST(APPEND SAMPLE_LIST ${PROJECT_NAME})
    SET(SAMPLE_LIST "${SAMPLE_LIST}" PARENT_SCOPE)
endif()

add_executable(cone_subscriber src/cone_subscriber.cpp)
ament_target_dependencies(cone_subscriber
  rclcpp
  std_msgs
  moa_msgs
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  moa_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  cone_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
