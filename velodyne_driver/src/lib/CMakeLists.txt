cmake_minimum_required(VERSION 3.5.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(gtest REQUIRED)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${velodyne_msgs_INCLUDE_DIRS}
)

add_library(velodyne_input input.cc)
target_link_libraries(velodyne_input
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${libpcap_LIBRARIES}
)

install(TARGETS velodyne_input
        LIBRARY DESTINATION lib
)
