##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(control_interpreter ${SOURCES})
add_dependencies(control_interpreter antdroid_msgs_gencpp)
target_link_libraries(control_interpreter ${catkin_LIBRARIES})

#install(TARGETS control_interpreter
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS control_interpreter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

