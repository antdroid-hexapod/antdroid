##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(cam_control ${SOURCES})
add_dependencies(cam_control sensor_msgs_gencpp)
target_link_libraries(cam_control ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


## Mark executables and/or libraries for installation
install(TARGETS cam_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

