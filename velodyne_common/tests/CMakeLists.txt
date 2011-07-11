# unit test uses gtest, but needs rostest to create a ROS environment
#rosbuild_add_executable(unit_test unit_test.cpp)
#target_link_libraries(unit_test camera_info_manager)
#rosbuild_add_gtest_build_flags(unit_test)

rosbuild_download_test_data(
  http://pr.willowgarage.com/data/velodyne/class.pcap
  tests/class.pcap
  65808d25772101358a3719b451b3d015
  )
