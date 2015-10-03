cmake_minimum_required(VERSION 2.8.3)
project(velodyne_gps_imu)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") #or -std=c++11

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  nmea_msgs
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velodyne_gps_imu
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(gpsimu_driver src/gpsimu_driver.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(gpsimu_driver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS gpsimu_driver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

