cmake_minimum_required(VERSION 3.5)
project(velodyne_laserscan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/node.cpp
  src/velodyne_laserscan.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# if (CATKIN_ENABLE_TESTING)
#   add_subdirectory(tests)
# endif()

ament_package()
