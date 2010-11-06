rosbuild_add_library(velodyne data.cc input.cc)
target_link_libraries(velodyne pcap)
