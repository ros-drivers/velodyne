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

add_library(velodyne_cloud_types SHARED
  pointcloudXYZIR.cpp
  organized_cloudXYZIR.cpp
)
ament_target_dependencies(velodyne_cloud_types
  rclcpp
  sensor_msgs
  tf2
  velodyne_msgs
)

install(TARGETS velodyne_cloud_types velodyne_rawdata
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
