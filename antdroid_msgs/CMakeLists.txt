cmake_minimum_required(VERSION 2.8.3)
project(antdroid_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation std_msgs
)

add_message_files(
    DIRECTORY   msg
        FILES   Balance.msg
                Calibrate.msg
                Foot.msg
                Gait.msg
                Height.msg
                Log.msg
                MoveLeg.msg
                Rotate.msg
                Speed.msg
                Walk.msg
)



generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
)
