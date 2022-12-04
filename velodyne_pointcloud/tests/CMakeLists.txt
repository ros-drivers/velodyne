# these dependencies are only needed for unit testing
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_row_step test_row_step.cpp)
ament_target_dependencies(test_row_step
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
  velodyne_msgs
)

ament_add_gtest(test_calibration
  test_calibration.cpp
)

target_link_libraries(test_calibration
  velodyne_rawdata
)

ament_target_dependencies(test_calibration
  rclcpp
)

target_link_libraries(test_calibration
  ${YAML_CPP_LIBRARIES}
)

# # Download packet capture (PCAP) files containing test data.
# # Store them in devel-space, so rostest can easily find them.
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
#   ${PROJECT_NAME}_tests_64e_s2.1-300-sztaki.pcap
#   http://download.ros.org/data/velodyne/64e_s2.1-300-sztaki.pcap
#   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#   MD5 176c900ffb698f9b948a13e281ffc1a2)
# catkin_download_test_data(
#   ${PROJECT_NAME}_tests_vlp16.pcap
#   http://download.ros.org/data/velodyne/vlp16.pcap
#   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#   MD5 f45c2bb1d7ee358274e423ea3b66fd73)

# run rostests
# add_rostest(transform_node_hz.test)
# add_rostest(transform_nodelet_hz.test)
# add_rostest(transform_node_32e_hz.test)
# add_rostest(transform_nodelet_32e_hz.test)
# add_rostest(transform_node_64e_s2.1_hz.test)
# add_rostest(transform_nodelet_64e_s2.1_hz.test)
# add_rostest(transform_node_vlp16_hz.test)
# add_rostest(transform_nodelet_vlp16_hz.test)
# add_rostest(two_nodelet_managers.test)

# # parse check all the launch/*.launch files
# roslaunch_add_file_check(../launch)
