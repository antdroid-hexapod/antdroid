/* controlRos.cpp: ROS control for hexapods
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

/*
 * If this library is included in the Antfirm program, it allows us
 * to control a hexapod by using the serial port of the Arduino MEGA
 * board like a ROS node. Warn: It is not compatible with serial control
 * system.
 *
 *The methos are:
 *
 * Control: Class for manipulating a hexapod by using the serial port
 * Start(): create node and subscribers
 * ReadInput(): wait to read orders publicated in topics.
 * 
 * RunCommand(): main menu for the rest of the methods.
 *
 * ControlWalk(antdroid_msgs::Walk& ): moves the hexapod to the selected 
 * (X, Y) coordinate.
 *
 * ControlBalance(antdroid_msgs::Balance& ): balances the hexapod 
 *
 * ControlRotate(antdroid_msgs::Rotate& ): spins the hexapod the selected 
 * number of degrees.
 *
 * ControlChangeSpeed(antdroid_msgs::Speed& ): changes the movement speed of
 * the hexapod [0 - 250]
 * 
 * ControlChangeHeight(antdroid_msgs::Height& ): changes the current height
 * of the hexapod.
 *
 * ControlChangeFootDistance(antdroid_msgs::Foot& ): changes foot distance of
 * hexapod's legs.
 *
 * ControlChangeLogLevel(const antdroid_msgs::Log& msg): changes log level
 *
 * ControlChangeCalibration(const antdroid_msgs::Calibrate& msg): changes
 * calibration values
 *
 */

#include "Configuration.h"

#ifdef ControlRos
#include "controlRos.h"

ros::Subscriber<antdroid_msgs::Walk> walk("Walk", &ControlWalk);
ros::Subscriber<antdroid_msgs::Balance> balance("Balance", &ControlBalance);
ros::Subscriber<antdroid_msgs::Rotate> rotate("Rotate", &ControlRotate);
ros::Subscriber<antdroid_msgs::Speed> speed ("Speed", &ControlChangeSpeed);
ros::Subscriber<antdroid_msgs::Height> height("Height", &ControlChangeHeight);
ros::Subscriber<antdroid_msgs::Foot> footDistance("Foot", &ControlChangeFootDistance);
ros::Subscriber<antdroid_msgs::Log> logLevel("Log", &ControlChangeLogLevel);
ros::Subscriber<antdroid_msgs::Calibrate> calibration("Calibrate", &ControlChangeCalibration);
ros::Subscriber<antdroid_msgs::Gait> gait("Gait", &ControlChangeGait);
ros::Subscriber<antdroid_msgs::MoveLeg> moveLeg("MoveLeg", &ControlMoveLeg);
ros::Subscriber<std_msgs::Bool> attack("Attack", &ControlAttack);


std_msgs::Bool is_new_message;
ros::Publisher pub_is_new_message("NewMessage", &is_new_message);

Control::Control(Hexapod* Antdroid)                                   
{
    is_new_message.data = true;
}

void Control::Start(void)
{
    arduino.initNode();

    arduino.subscribe(walk);
    arduino.subscribe(balance);
    arduino.subscribe(rotate);
    arduino.subscribe(speed);
    arduino.subscribe(height);
    arduino.subscribe(footDistance);
    arduino.subscribe(logLevel);
    arduino.subscribe(calibration);
    arduino.subscribe(gait);
    arduino.subscribe(moveLeg);
    arduino.subscribe(attack);

    arduino.advertise(pub_is_new_message);

    level_log = 0;

}

void Control::ReadInput(void)
{
    arduino.spinOnce();
}

void ControlWalk(const antdroid_msgs::Walk& msg)
{
    Antdroid.Walk(msg.x, msg.y);

    is_new_message.data = true;
    pub_is_new_message.publish(&is_new_message);
    pub_is_new_message.publish(&is_new_message);
}


void ControlBalance(const antdroid_msgs::Balance& msg)
{
    Antdroid.Balance(msg.pitch, msg.roll, msg.yaw);
}

void ControlRotate(const antdroid_msgs::Rotate& msg)
{
    Antdroid.Rotate(msg.yaw);

    is_new_message.data = true;
    pub_is_new_message.publish(&is_new_message);
    pub_is_new_message.publish(&is_new_message);
}

void ControlChangeSpeed(const antdroid_msgs::Speed& msg)
{
    Antdroid.ChangeSpeed(msg.speed);
}

void ControlChangeHeight(const antdroid_msgs::Height& msg)
{
    Antdroid.ChangeHeight(msg.height);
}

void ControlChangeFootDistance(const antdroid_msgs::Foot& msg)
{
    Antdroid.ChangeFootDistance(msg.footDistance);
}

void ControlChangeLogLevel(const antdroid_msgs::Log& msg)
{
    level_log = msg.level;
}

void ControlChangeCalibration(const antdroid_msgs::Calibrate& msg)
{
    Antdroid.CalibrateLeg(msg.leg, msg.member, msg.angle);
}

void ControlChangeGait(const antdroid_msgs::Gait& msg)
{
    const uint8_t sequence[6]= { msg.leg0, msg.leg1, msg.leg2, msg.leg3, msg.leg4,
        msg.leg5};

    Antdroid.ChangeGait(msg.type, sequence);

    is_new_message.data = false;
    pub_is_new_message.publish(&is_new_message);
}


void ControlMoveLeg(const antdroid_msgs::MoveLeg& msg)
{
    Antdroid.MoveLeg(msg.leg, msg.x, msg.y, msg.z);
}

void ControlAttack(const std_msgs::Bool& msg)
{
     Antdroid.Attack();
}

#endif