
rosbuild_add_executable(cloud cloud.cc)
include_directories(cloud ${PROJECT_NAME}/include)
#link_directories(cloud ${PROJECT_NAME}/lib)
target_link_libraries(cloud pcap velodyne)

rosbuild_add_executable(print print.cc)
include_directories(print ${PROJECT_NAME}/include)
#link_directories(print ${PROJECT_NAME}/lib)
target_link_libraries(print pcap velodyne)

rosbuild_add_executable(read read.cc)
include_directories(read ${PROJECT_NAME}/include)
#link_directories(read ${PROJECT_NAME}/lib)
target_link_libraries(read pcap velodyne)
