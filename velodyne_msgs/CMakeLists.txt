cmake_minimum_required(VERSION 3.5)

project(velodyne_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(tinyxml2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/VelodynePacket.msg"
   "msg/VelodyneScan.msg"
   DEPENDENCIES std_msgs
   DEPENDENCIES builtin_interfaces

)

ament_package()