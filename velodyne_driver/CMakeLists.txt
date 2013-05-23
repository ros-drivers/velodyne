cmake_minimum_required(VERSION 2.8.3)
project(velodyne_driver)

set(${PROJECT}_CATKIN_DEPS 
    diagnostic_updater
    nodelet
    roscpp
    tf
    velodyne_msgs)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT}_CATKIN_DEPS} rostest)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# objects needed by other ROS packages that depend on this one
catkin_package(CATKIN_DEPENDS ${${PROJECT}_CATKIN_DEPS}
               INCLUDE_DIRS include
               LIBRARIES velodyne_input)

# compile the driver and input library
add_subdirectory(src/lib)
add_subdirectory(src/driver)

# Download packet capture (PCAP) files containing test data.
#
# This appalling hack is to work around download_test_data(), which
# insists on copying files to the build tree, where rostest cannot
# find them.
set(SAVE_PROJECT_BINARY_DIR ${PROJECT_BINARY_DIR})
set(PROJECT_BINARY_DIR /tmp/velodyne_driver)
download_test_data(http://pr.willowgarage.com/data/velodyne/class.pcap
                   tests/class.pcap
                   65808d25772101358a3719b451b3d015)

download_test_data(http://pr.willowgarage.com/data/velodyne/32e.pcap
                   tests/32e.pcap
                   e41d02aac34f0967c03a5597e1d554a9)
set(PROJECT_BINARY_DIR ${SAVE_PROJECT_BINARY_DIR})

# unit tests
# (not working yet: can't find the test data)
add_rostest(tests/pcap_node_hertz.test)
add_rostest(tests/pcap_nodelet_hertz.test)
add_rostest(tests/pcap_32e_node_hertz.test)
add_rostest(tests/pcap_32e_nodelet_hertz.test)

# parse check all the launch/*.launch files
#rosbuild_add_roslaunch_check(launch)
