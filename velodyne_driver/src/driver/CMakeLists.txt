cmake_minimum_required(VERSION 3.5.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(gtest REQUIRED)
find_package(console_bridge REQUIRED)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
                    ${console_bridge_INCLUDE_DIRS}
                    ${diagnostic_updater_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${velodyne_msgs_INCLUDE_DIRS}
                    ${diagnostic_msgs_INCLUDE_DIRS}
)
#${rclcpp_components_INCLUDE_DIRS}

# build the driver node
add_executable(velodyne_node velodyne_node.cc driver.cc)

target_link_libraries(velodyne_node
  velodyne_input
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${velodyne_msgs_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES}
  ${libpcap_LIBRARIES}
)

ament_target_dependencies(velodyne_node
  velodyne_input
  "rclcpp"
  "rmw_implementation"
  "velodyne_msgs"
  "diagnostic_msgs"
  "libpcap"
)

# install runtime and library files
install(TARGETS velodyne_node
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)
