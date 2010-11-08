
rosbuild_add_library(driver_nodelet driver_nodelet.cc)
include_directories(driver_nodelet ${PROJECT_NAME}/include)
link_directories(driver_nodelet ${PROJECT_NAME}/lib)
target_link_libraries(driver_nodelet pcap velodyne)

rosbuild_add_library(pointcloud_nodelet pointcloud_nodelet.cc)
include_directories(pointcloud_nodelet ${PROJECT_NAME}/include)
link_directories(pointcloud_nodelet ${PROJECT_NAME}/lib)
target_link_libraries(pointcloud_nodelet pcap velodyne)
