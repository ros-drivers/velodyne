add_library(velodyne_rawdata rawdata.cc calibration.cc)
target_link_libraries(velodyne_rawdata 
  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
