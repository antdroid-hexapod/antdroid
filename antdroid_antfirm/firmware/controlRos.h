/* controlRos.h: ROS control for hexapods
 *
 * Copyright (C) 2014 Alexander Gil and Javier Román
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef control_h
#define control_h

#include <ArduinoHardware.h>
#include "ros.h"

#include "hexapod.h"
#include "Arduino.h"


#include <antdroid_msgs/Walk.h>
#include <antdroid_msgs/Balance.h>
#include <antdroid_msgs/Rotate.h>
#include <antdroid_msgs/Speed.h>
#include <antdroid_msgs/Height.h>
#include <antdroid_msgs/Foot.h>
#include <antdroid_msgs/Log.h>
#include <antdroid_msgs/Calibrate.h>
#include <antdroid_msgs/Gait.h>
#include <antdroid_msgs/MoveLeg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

extern ros::NodeHandle arduino;
extern Hexapod Antdroid;

void ControlWalk(const antdroid_msgs::Walk& msg);
void ControlBalance(const antdroid_msgs::Balance& msg);
void ControlRotate(const antdroid_msgs::Rotate& msg);
void ControlChangeSpeed(const antdroid_msgs::Speed& msg);
void ControlChangeHeight(const antdroid_msgs::Height& msg);
void ControlChangeFootDistance(const antdroid_msgs::Foot& msg);
void ControlChangeLogLevel(const antdroid_msgs::Log& msg);
void ControlChangeCalibration(const antdroid_msgs::Calibrate& msg);
void ControlChangeGait(const antdroid_msgs::Gait& msg);
void ControlMoveLeg(const antdroid_msgs::MoveLeg& msg);
void ControlAttack(const std_msgs::Bool& msg);
void ControlSayHello(const std_msgs::Bool& msg);

class Control
{
    public:
    Control(Hexapod* Antdroid);
    
    void Start(void);

    void ReadInput(void);

};

#endif