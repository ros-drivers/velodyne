cmake_minimum_required(VERSION 3.5)
project(velodyne_pcl)

find_package(catkin REQUIRED COMPONENTS pcl_ros roslint)

roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

