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

# unit tests
#
# $ROS_BUILD_TEST_LABEL controls which label "make test" runs
#
#  * if unset, all tests run
#  * tests labelled "pcap" require more processor time than the
#    ROS build machines can reliably guarantee.
#  * only unlabelled tests or those labelled "hudson" automatically
#    run on the ROS build machines
#
#add_subdirectory(tests)
#rosbuild_add_rostest_labeled(pcap tests/pcap_node_hertz.test)
#rosbuild_add_rostest_labeled(pcap tests/pcap_nodelet_hertz.test)
#rosbuild_add_rostest_labeled(pcap tests/pcap_32e_node_hertz.test)
#rosbuild_add_rostest_labeled(pcap tests/pcap_32e_nodelet_hertz.test)

# parse check all the launch/*.launch files
#rosbuild_add_roslaunch_check(launch)
