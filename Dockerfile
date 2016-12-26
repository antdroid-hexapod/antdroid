FROM pando85/armhf-ros-indigo

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-indigo-ros-comm ros-indigo-diagnostics ros-indigo-joystick-drivers ros-indigo-rosserial ros-indigo-rosserial-server ros-indigo-rosserial-arduino ros-indigo-robot-upstart ros-indigo-bond-core ros-indigo-dynamic-reconfigure ros-indigo-nodelet-core ros-indigo-class-loader ros-indigo-image-common ros-indigo-vision-opencv ros-indigo-image-transport-plugins ros-indigo-angles \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    arduino \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src && \
    /bin/bash -c 'source /opt/ros/indigo/setup.bash; catkin_init_workspace /catkin_ws/src'

RUN /bin/bash -c 'source /opt/ros/indigo/setup.bash; cd /catkin_ws; catkin_make'

ADD . /catkin_ws/src/antdroid

RUN /bin/bash -c 'source /opt/ros/indigo/setup.bash; cd /catkin_ws; catkin_make'

## Remove PORT
#RUN sed -i "s%PORT /dev/ttyACM0%%g" /catkin_ws/src/antdroid/antdroid_antfirm/firmware/CMakeLists.txt
