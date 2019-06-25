add_executable(cloud_node cloud_node.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
target_link_libraries(cloud_node velodyne_rawdata ${YAML_CPP_LIBRARIES})
ament_target_dependencies(cloud_node
  Eigen3
  rclcpp
  tf2
  velodyne_msgs
)
install(TARGETS cloud_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(transform_node transform_node.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
target_link_libraries(transform_node velodyne_rawdata ${YAML_CPP_LIBRARIES})
ament_target_dependencies(transform_node
  message_filters
  rclcpp
  tf2
  tf2_ros
)
install(TARGETS transform_node
        DESTINATION lib/${PROJECT_NAME})
