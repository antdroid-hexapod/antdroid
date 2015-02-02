/* antdroid_joy.cpp: provides control of antdroid robot using keyboard or joy.
 *
 * Copyright (C) 2015 Alexander Gil and Javier Román
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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "../include/antdroid_teleop/antdroid_joy.hpp"

AntdroidTeleop::AntdroidTeleop():
    _ph("~"),
    _linear(PS3_AXIS_STICK_LEFT_UPWARDS),
    _angular(PS3_AXIS_STICK_LEFT_LEFTWARDS),
    _walk_button(PS3_BUTTON_REAR_LEFT_1),
    _rotate_left_button(PS3_AXIS_BUTTON_REAR_LEFT_2),
    _rotate_right_button(PS3_AXIS_BUTTON_REAR_RIGHT_2),
    _rise_speed_button(PS3_BUTTON_CROSS_UP),
    _decrease_speed_button(PS3_BUTTON_CROSS_DOWN),
    _l_scale(1),
    _a_scale(1)
{
    _ph.param("axis_linear", _linear, _linear);
    _ph.param("axis_angular", _angular, _angular);
    _ph.param("axis_deadman", _walk_button, _walk_button);
    _ph.param("scale_angular", _a_scale, _a_scale);
    _ph.param("scale_linear", _l_scale, _l_scale);

    _walk_mode = false;
    _change_speed.speed = CONST_SPEED;

    _vel_pub = _ph.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    _speed_pub = _ph.advertise<antdroid_msgs::Speed>("speed", 1, true);

    _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 10,
        &AntdroidTeleop::joyCallback, this);

    _timer = _nh.createTimer(ros::Duration(0.1), boost::bind(&AntdroidTeleop::publish, this));
}

void AntdroidTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
    geometry_msgs::Twist vel;
    vel.linear.y = _l_scale*joy->axes[_angular];
    vel.linear.x = _l_scale*joy->axes[_linear];


    if(joy->buttons[_rotate_left_button])
    {
        vel.angular.z = 1;
    }
    else if(joy->buttons[_rotate_right_button])
    {
        vel.angular.z = -1;
    }
    else 
    {
        vel.angular.z = 0;
    }

    if(joy->buttons[_rise_speed_button])
    {
        _change_speed.speed = RISE_SPEED;
    }
    else if(joy->buttons[_decrease_speed_button])
    {
        _change_speed.speed = DECREASE_SPEED;
    }
    else 
    {
        _change_speed.speed = CONST_SPEED;
    }



    _last_published = vel;
    _walk_mode = joy->buttons[_walk_button];
}

void AntdroidTeleop::publish()
{
    boost::mutex::scoped_lock lock(_publish_mutex);

    if (_walk_mode && (_last_published.linear.x != 0 ) && (_last_published.linear.y != 0))
    {
        _vel_pub.publish(_last_published);
    }
    else if(!_walk_mode && (_last_published.angular.z != 0))
    {
        _vel_pub.publish(_last_published);
    }

    if(_change_speed.speed != CONST_SPEED)
        _speed_pub.publish(_change_speed);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "antdroid_teleop");
    AntdroidTeleop antdroid_teleop;

    ros::spin();
}