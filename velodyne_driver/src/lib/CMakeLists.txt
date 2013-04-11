add_library(velodyne_input input.cc)
target_link_libraries(velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
install(TARGETS velodyne_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
