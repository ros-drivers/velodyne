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
install(TARGETS velodyne_rawdata
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
