cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)


generate_arduino_firmware(antfirm
    SRCS Antfirm.cpp calibration.cpp controlRos.cpp controlSerial.cpp hexapod.cpp leg.cpp log.cpp trig.cpp voltageMonitor.cpp ${ROS_LIB_DIR}/time.cpp ${ROS_LIB_DIR}/duration.cpp
    HDRS boards.h Configuration.h calibration.h controlRos.h controlSerial.h hexapod.h leg.h log.h pins.h trig.h uncopyable.h voltageMonitor.h
    BOARD mega2560
    PORT /dev/ttyUSB0
)