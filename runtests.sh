#!/bin/bash

docker build  . -t pando85/antdroid && docker run --rm pando85/antdroid /bin/bash -c 'source /opt/ros/indigo/setup.bash; cd /catkin_ws; catkin_make antdroid_antfirm_firmware_upload || rm -rf build && catkin_make antdroid_antfirm_firmware_upload'
