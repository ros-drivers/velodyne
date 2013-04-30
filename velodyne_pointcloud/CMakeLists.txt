cmake_minimum_required(VERSION 2.8.3)
project(velodyne_pointcloud)
find_package(catkin REQUIRED
  COMPONENTS
  nodelet
  pcl_ros
  roscpp
  roslib
  rostest
  sensor_msgs
  tf
  velodyne_driver
  velodyne_msgs
)
find_package(Boost COMPONENTS signals)
find_package(pcl)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src/lib)
add_subdirectory(src/conversions)

catkin_package(
    CATKIN_DEPENDS nodelet pcl_ros roscpp roslib sensor_msgs tf velodyne_driver velodyne_msgs
    DEPENDS yaml-cpp python-yaml
    INCLUDE_DIRS include
    LIBRARIES velodyne_rawdata
)

### install some launch scripts and parameter files

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
#rosbuild_add_rostest_labeled(pcap tests/cloud_node_hz.test)
#rosbuild_add_rostest_labeled(pcap tests/cloud_nodelet_hz.test)
#rosbuild_add_rostest_labeled(pcap tests/cloud_node_32e_hz.test)
#rosbuild_add_rostest_labeled(pcap tests/cloud_nodelet_32e_hz.test)
#rosbuild_add_rostest_labeled(pcap tests/transform_node_hz.test)
#rosbuild_add_rostest_labeled(pcap tests/transform_nodelet_hz.test)

# parse check all the launch/*.launch files
#rosbuild_add_roslaunch_check(launch)
