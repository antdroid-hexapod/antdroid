cmake_minimum_required(VERSION 2.8.3)
project(antdroid_antfirm)

find_package(catkin REQUIRED COMPONENTS
    rosserial_arduino
    rosserial_client
    antdroid_msgs
    std_msgs
)


#add_message_files(
#    FILES
#    Balance.msg
#    Foot.msg
#    Height.msg
#    Rotate.msg
#    Speed.msg
#    Walk.msg
#)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   antdroid_msgs
 )


catkin_package()


rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
    )

rosserial_configure_client(
    DIRECTORY firmware
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
    )
 
rosserial_add_client_target(firmware ALL)
rosserial_add_client_target(firmware upload)