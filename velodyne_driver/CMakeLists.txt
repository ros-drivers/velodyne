cmake_minimum_required(VERSION 3.5)
project(velodyne_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(velodyne_msgs REQUIRED)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

# compile the driver and input library
add_library(velodyne_input SHARED src/lib/input.cpp)
target_include_directories(velodyne_input PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_input PUBLIC
  ${libpcap_LIBRARIES}
  rclcpp::rclcpp
  ${velodyne_msgs_TARGETS}
)

# build the driver node
add_library(velodyne_driver SHARED src/driver/driver.cpp)
target_include_directories(velodyne_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_driver PUBLIC
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  velodyne_input
  ${velodyne_msgs_TARGETS}
)
target_link_libraries(velodyne_driver PRIVATE
  rclcpp_components::component
  tf2_ros::tf2_ros
)

add_executable(velodyne_driver_node src/driver/velodyne_node.cpp)
target_link_libraries(velodyne_driver_node PRIVATE rclcpp::rclcpp velodyne_driver)

# install runtime and library files
install(TARGETS velodyne_input velodyne_driver
  EXPORT velodyne_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS velodyne_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(velodyne_driver
  "velodyne_driver::VelodyneDriver")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS src/vdump
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # unit test
  ament_add_gtest(time_test tests/timeconversiontest.cpp)
  target_link_libraries(time_test velodyne_input)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(velodyne_driver)
ament_export_dependencies(
  diagnostic_updater
  rclcpp
  velodyne_msgs
)

ament_package()
