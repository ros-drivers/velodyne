add_library(data_containers pointcloudXYZIRT.cc organized_cloudXYZIRT.cc)
add_dependencies(data_containers ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(data_containers velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(transform_node transform_node.cc transform.cc)
add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(transform_nodelet transform_nodelet.cc transform.cc)
add_dependencies(transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS data_containers transform_node transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
