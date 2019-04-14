cmake_minimum_required(VERSION 3.5)
project(velodyne_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rcl REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(gtest REQUIRED)
find_package(console_bridge REQUIRED)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
                    ${console_bridge_INCLUDE_DIRS}
                    ${diagnostic_msgs_INCLUDE_DIRS}
                    ${diagnostic_updater_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${velodyne_msgs_INCLUDE_DIRS}
                    )

# compile the driver and input library
add_subdirectory(src/lib)
add_subdirectory(src/driver)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)


install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS src/vdump
        DESTINATION lib/${PROJECT_NAME})


#if (BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  # these dependencies are only needed for unit testing
  # find_package(roslaunch REQUIRED)
  # find_package(rostest REQUIRED)

  # Download packet capture (PCAP) files containing test data.
  # Store them in devel-space, so rostest can easily find them.
  # catkin_download_test_data(
  # ${PROJECT_NAME}_tests_class.pcap
  # http://download.ros.org/data/velodyne/class.pcap
  # DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  # MD5 65808d25772101358a3719b451b3d015)
  # catkin_download_test_data(
  # ${PROJECT_NAME}_tests_32e.pcap
  # http://download.ros.org/data/velodyne/32e.pcap
  # DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  # MD5 e41d02aac34f0967c03a5597e1d554a9)
  # catkin_download_test_data(
  # ${PROJECT_NAME}_tests_vlp16.pcap
  # http://download.ros.org/data/velodyne/vlp16.pcap
  # DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
  # MD5 f45c2bb1d7ee358274e423ea3b66fd73)
  
  # unit tests
  # add_rostest(tests/pcap_node_hertz.test)
  # add_rostest(tests/pcap_nodelet_hertz.test)
  # add_rostest(tests/pcap_32e_node_hertz.test)
  # add_rostest(tests/pcap_32e_nodelet_hertz.test)
  # add_rostest(tests/pcap_vlp16_node_hertz.test)
  # add_rostest(tests/pcap_vlp16_nodelet_hertz.test)
  
  # parse check all the launch/*.launch files
  # roslaunch_add_file_check(launch)

  # unit test
  # catkin_add_gtest(time_test tests/timeconversiontest.cpp)
  # target_link_libraries(time_test
  # ${catkin_LIBRARIES}
#endif (CATKIN_ENABLE_TESTING)

ament_package()
