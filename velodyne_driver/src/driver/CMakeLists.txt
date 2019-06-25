# build the driver node
add_executable(velodyne_node velodyne_node.cc driver.cc)

ament_target_dependencies(velodyne_node
  diagnostic_msgs
  rclcpp
  tf2_ros
  velodyne_msgs
)
target_link_libraries(velodyne_node velodyne_input)

# install runtime and library files
install(TARGETS velodyne_node
        DESTINATION lib/${PROJECT_NAME}
)
