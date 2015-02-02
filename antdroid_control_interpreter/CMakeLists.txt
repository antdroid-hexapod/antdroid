cmake_minimum_required(VERSION 2.8.3)
project(antdroid_control_interpreter)

find_package(catkin REQUIRED COMPONENTS
  antdroid_msgs
  geometry_msgs
  std_msgs
  roscpp
)


#generate_messages(
#   DEPENDENCIES
#   antdroid_msgs
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES antdroid_control_interpreter
  CATKIN_DEPENDS antdroid_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
