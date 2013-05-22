cmake_minimum_required(VERSION 2.8.3)
project(velodyne_driver)
find_package(catkin REQUIRED
  COMPONENTS
  diagnostic_updater
  nodelet
  roscpp
  rostest
  tf
  velodyne_msgs
)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

#find_package(libpcap REQUIRED)
set(libpcap_LIBRARIES -lpcap)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# objects needed by other ROS packages that depend on this one
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES velodyne_input
)

# compile the driver and input library
add_subdirectory(src/lib)
add_subdirectory(src/driver)

# Download packet capture (PCAP) files containing test data.
download_test_data(http://pr.willowgarage.com/data/velodyne/class.pcap
                   tests/class.pcap
                   65808d25772101358a3719b451b3d015)

download_test_data(http://pr.willowgarage.com/data/velodyne/32e.pcap
                   tests/32e.pcap
                   e41d02aac34f0967c03a5597e1d554a9)

# unit tests
# (not working yet: can't find the test data)
#add_rostest(tests/pcap_node_hertz.test)
#add_rostest(tests/pcap_nodelet_hertz.test)
#add_rostest(tests/pcap_32e_node_hertz.test)
#add_rostest(tests/pcap_32e_nodelet_hertz.test)

# parse check all the launch/*.launch files
#rosbuild_add_roslaunch_check(launch)
