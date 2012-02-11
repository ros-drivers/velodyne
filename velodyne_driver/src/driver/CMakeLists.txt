
rosbuild_add_executable(velodyne_node velodyne_node.cc)
target_link_libraries(velodyne_node pcap)

rosbuild_add_library(driver_nodelet driver_nodelet.cc)
include_directories(driver_nodelet ${PROJECT_NAME}/include)
target_link_libraries(driver_nodelet pcap velodyne)
