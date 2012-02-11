cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

add_subdirectory(src/lib)
add_subdirectory(src/driver)

# driver unit tests
#
# $ROS_BUILD_TEST_LABEL controls which label "make test" runs
#
#  * if unset, all tests run
#
#  * tests labelled "realtime" use a downloaded pcap, but may not work
#    on heavily-loaded build machines
#
#  * only unlabelled tests or those labelled "hudson" automatically
#    run on the ROS build machines
#
##add_subdirectory(tests)
##rosbuild_add_rostest_labeled(realtime tests/pcap_hertz.test)
##rosbuild_add_rostest_labeled(realtime tests/pointcloud_nodelet_hertz.test)
##rosbuild_add_rostest_labeled(realtime tests/transform_hertz.test)
