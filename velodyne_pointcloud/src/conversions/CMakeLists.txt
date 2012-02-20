
rosbuild_add_executable(cloud2_node cloud2_node.cc convert.cc)
target_link_libraries(cloud2_node velodyne_rawdata)

rosbuild_add_library(cloud2_nodelet cloud2_nodelet.cc convert.cc)
target_link_libraries(cloud2_nodelet velodyne_rawdata)

rosbuild_add_library(transform2 transform2.cc)
