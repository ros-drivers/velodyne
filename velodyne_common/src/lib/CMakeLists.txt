rosbuild_add_library(velodyne
  data.cc
  data_base.cc data_scans.cc data_xyz.cc
  input.cc)
target_link_libraries(velodyne pcap)
