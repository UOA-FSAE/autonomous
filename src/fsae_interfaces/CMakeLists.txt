cmake_minimum_required(VERSION 3.5)
project(fsae_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AllStates.msg"
  "msg/AllTrajectories.msg"
  "msg/BoundaryStamped.msg"
  "msg/CAN.msg"
  "msg/CANStamped.msg"
  "msg/Cone.msg"
  "msg/ConeMap.msg"
  "msg/ConeMapStamped.msg"
  "msg/ConeStamped.msg"
  "msg/Detections.msg"
  "msg/FusedCone.msg"
  "msg/FusedCones.msg"
  "msg/HardwareStates.msg"
  "msg/HardwareStatesStamped.msg"
  "msg/MissionStates.msg"
  "msg/MissionStatesStamped.msg"
  "msg/OccupancyGrid.msg"
  "msg/Pulse.msg"
  "msg/Track.msg"
)

set(srv_files
  "srv/CANSendReq.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs ackermann_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
