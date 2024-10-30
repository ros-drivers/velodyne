# these dependencies are only needed for unit testing
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)

ament_add_gtest(test_row_step test_row_step.cpp)
target_include_directories(test_row_step PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(test_row_step
  Eigen3::Eigen
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${velodyne_msgs_TARGETS}
)

ament_add_gtest(test_calibration
  test_calibration.cpp
)
target_link_libraries(test_calibration
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  velodyne_rawdata
)
