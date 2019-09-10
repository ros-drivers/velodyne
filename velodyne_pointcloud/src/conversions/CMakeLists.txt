add_library(data_containers pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(data_containers ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(data_containers velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(cloud_node cloud_node.cc convert.cc)
add_dependencies(cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_node velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(cloud_nodelet cloud_nodelet.cc convert.cc)
add_dependencies(cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_nodelet velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(transform_node transform_node.cc transform.cc)
add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(transform_nodelet transform_nodelet.cc transform.cc)
add_dependencies(transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(TARGETS data_containers cloud_node cloud_nodelet transform_node transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
