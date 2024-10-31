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
find_package(diagnostic_updater REQUIRED)
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
find_package(yaml-cpp REQUIRED)

# Older versions of yaml-cpp have a target called "yaml-cpp",
# so use that where appropriate.
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET "yaml-cpp::yaml-cpp")
else()
  set(YAML_CPP_TARGET ${YAML_CPP_LIBRARIES})
endif()

# Work around broken find module in AlmaLinux/RHEL eigen3-devel from PowerTools repo
find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 REQUIRED)
endif()

add_library(velodyne_rawdata SHARED
  src/lib/rawdata.cpp
  src/lib/calibration.cpp)
target_include_directories(velodyne_rawdata PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_rawdata PUBLIC
  angles::angles
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${PCL_LIBRARIES}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${velodyne_msgs_TARGETS}
  ${YAML_CPP_TARGET}
)

add_library(velodyne_cloud_types SHARED
  src/lib/pointcloudXYZIRT.cpp
  src/lib/organized_cloudXYZIRT.cpp
)
target_include_directories(velodyne_cloud_types PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_cloud_types PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${velodyne_msgs_TARGETS}
)

add_library(transform SHARED
  src/conversions/transform.cpp)
target_include_directories(transform PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(transform PUBLIC
  diagnostic_updater::diagnostic_updater
  Eigen3::Eigen
  message_filters::message_filters
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  velodyne_cloud_types
  velodyne_rawdata
  ${YAML_CPP_TARGET}
)
target_link_libraries(transform PRIVATE
  rclcpp_components::component
)

install(TARGETS velodyne_cloud_types velodyne_rawdata transform
  EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

rclcpp_components_register_nodes(transform
  "velodyne_pointcloud::Transform")

add_executable(velodyne_transform_node
  src/conversions/transform_node.cpp)
target_include_directories(velodyne_transform_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(velodyne_transform_node PRIVATE
  rclcpp::rclcpp
  transform
)
install(TARGETS velodyne_transform_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Remove empty.xml from lint
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_xmllint
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE XML_FILES
    "*.xml"
  )

  get_filename_component(
    EMPTY_XML_ABS
    "tests/empty.xml"
    ABSOLUTE
  )

  list(REMOVE_ITEM XML_FILES ${EMPTY_XML_ABS})

  find_package(ament_cmake_xmllint REQUIRED)

  ament_xmllint(${XML_FILES})

  add_subdirectory(tests)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  diagnostic_updater
  eigen
  geometry_msgs
  message_filters
  pcl
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  velodyne_msgs
)
ament_export_libraries(velodyne_rawdata velodyne_cloud_types)
ament_export_targets(${PROJECT_NAME})

ament_package()
