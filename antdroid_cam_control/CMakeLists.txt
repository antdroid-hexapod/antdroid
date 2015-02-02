cmake_minimum_required(VERSION 2.8.3)
project(antdroid_cam_control)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)




catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs   geometry_msgs
  DEPENDS OpenCV
)



include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

#target_link_libraries(antdroid_cam_control)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)


#install(TARGETS antdroid_cam_control
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
