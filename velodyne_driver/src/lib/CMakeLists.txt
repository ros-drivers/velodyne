add_library(velodyne_input SHARED input.cpp)
ament_target_dependencies(velodyne_input
  rclcpp
  velodyne_msgs
)
target_link_libraries(velodyne_input
  ${libpcap_LIBRARIES}
)

install(TARGETS velodyne_input
  LIBRARY DESTINATION lib
)
