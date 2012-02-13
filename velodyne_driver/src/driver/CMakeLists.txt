
rosbuild_add_executable(velodyne_node velodyne_node.cc driver.cc)
target_link_libraries(velodyne_node velodyne_input pcap)

rosbuild_add_library(driver_nodelet nodelet.cc driver.cc)
target_link_libraries(driver_nodelet velodyne_input pcap)
