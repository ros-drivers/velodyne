# nodes
rosbuild_add_executable(cloud2 cloud2.cc)

# nodelets
rosbuild_add_library(cloud2_nodelet cloud2_nodelet.cc)
rosbuild_add_library(transform2 transform2.cc)
rosbuild_add_library(ring_colors ring_colors.cc)
