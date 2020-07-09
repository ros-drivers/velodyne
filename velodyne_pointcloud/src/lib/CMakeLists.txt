add_library(velodyne_rawdata SHARED
  rawdata.cpp
  calibration.cpp)
ament_target_dependencies(velodyne_rawdata
  angles
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  velodyne_msgs
)
target_link_libraries(velodyne_rawdata
  ${PCL_LIBRARIES}
)

target_link_libraries(velodyne_rawdata
        ${YAML_CPP_LIBRARIES})

add_library(velodyne_cloud_types SHARED
  pointcloudXYZIRT.cpp
  organized_cloudXYZIRT.cpp
)
ament_target_dependencies(velodyne_cloud_types
  rclcpp
  sensor_msgs
  tf2_ros
  velodyne_msgs
)
target_link_libraries(velodyne_cloud_types
  ${PCL_LIBRARIES}
)

install(TARGETS velodyne_cloud_types velodyne_rawdata
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
