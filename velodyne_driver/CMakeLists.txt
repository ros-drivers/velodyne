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
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(velodyne_msgs REQUIRED)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include)

# compile the driver and input library
add_subdirectory(src/lib)

# build the driver node
add_library(velodyne_driver SHARED src/driver/driver.cpp)
ament_target_dependencies(velodyne_driver
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  tf2_ros
  velodyne_msgs
)
target_link_libraries(velodyne_driver velodyne_input)

add_executable(velodyne_driver_node src/driver/velodyne_node.cpp)
ament_target_dependencies(velodyne_driver_node
  rclcpp
)
target_link_libraries(velodyne_driver_node velodyne_driver)

# install runtime and library files
install(TARGETS velodyne_driver
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

  # Download packet capture (PCAP) files containing test data.
  # Store them in devel-space, so rostest can easily find them.
  # catkin_download_test_data(
  #   ${PROJECT_NAME}_tests_class.pcap
  #   http://download.ros.org/data/velodyne/class.pcap
  #   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  #   MD5 65808d25772101358a3719b451b3d015)
  # catkin_download_test_data(
  #   ${PROJECT_NAME}_tests_32e.pcap
  #   http://download.ros.org/data/velodyne/32e.pcap
  #   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  #   MD5 e41d02aac34f0967c03a5597e1d554a9)
  # catkin_download_test_data(
  #   ${PROJECT_NAME}_tests_vlp16.pcap
  #   http://download.ros.org/data/velodyne/vlp16.pcap
  #   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  #   MD5 f45c2bb1d7ee358274e423ea3b66fd73)

  # unit tests
  # add_rostest(tests/pcap_node_hertz.test)
  # add_rostest(tests/pcap_nodelet_hertz.test)
  # add_rostest(tests/pcap_32e_node_hertz.test)
  # add_rostest(tests/pcap_32e_nodelet_hertz.test)
  # add_rostest(tests/pcap_vlp16_node_hertz.test)
  # add_rostest(tests/pcap_vlp16_nodelet_hertz.test)

  # parse check all the launch/*.launch files
  # roslaunch_add_file_check(launch)
endif()

ament_package()
