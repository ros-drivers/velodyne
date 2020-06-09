cmake_minimum_required(VERSION 3.5)
project(velodyne_pcl)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/velodyne_pcl.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${PCL_COMMON_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

