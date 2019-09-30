cmake_minimum_required(VERSION 3.5)
project(velodyne_pointcloud)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
# TODO(clalancette): not yet available
#find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(velodyne_msgs REQUIRED)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

add_subdirectory(src/lib)

add_library(convert SHARED src/conversions/convert.cc src/conversions/pointcloudXYZIR.cc src/conversions/organized_cloudXYZIR.cc)
ament_target_dependencies(convert
  Eigen3
  rclcpp
  rclcpp_components
  tf2
  velodyne_msgs
)
target_link_libraries(convert velodyne_rawdata ${YAML_CPP_LIBRARIES})
install(TARGETS convert
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
rclcpp_components_register_nodes(convert
  "velodyne_pointcloud::Convert")

add_executable(convert_node src/conversions/convert_node.cc)
ament_target_dependencies(convert_node
  rclcpp
)
target_link_libraries(convert_node convert)
install(TARGETS convert_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(transform SHARED src/conversions/transform.cc src/conversions/pointcloudXYZIR.cc src/conversions/organized_cloudXYZIR.cc)
ament_target_dependencies(transform
  message_filters
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
)
target_link_libraries(transform velodyne_rawdata ${YAML_CPP_LIBRARIES})
install(TARGETS transform
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
rclcpp_components_register_nodes(transform
  "velodyne_pointcloud::Transform")

add_executable(transform_node src/conversions/transform_node.cc)
ament_target_dependencies(transform_node
  rclcpp
)
target_link_libraries(transform_node transform)
install(TARGETS transform_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})
# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME})
# install(PROGRAMS scripts/gen_calibration.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
