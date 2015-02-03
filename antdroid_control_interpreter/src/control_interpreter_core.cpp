/* control_interpreter_core.cpp: control_interpreter_core of antdroid_control_parse node.
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
#include "../include/antdroid_control_interpreter/control_interpreter_core.hpp"

namespace control_interpreter_core
{


ControlInterpreterCore::ControlInterpreterCore() : 
    _walk(new antdroid_msgs::Walk()),
    _rotate(new antdroid_msgs::Rotate()),
    _step(20),
    _new_message_count(INIT_NEW_MESSAGE_COUNTER),
    _checker_count(0)
{

}

ControlInterpreterCore::~ControlInterpreterCore()
{

}

bool ControlInterpreterCore::init()
{
    ros::NodeHandle nh("control_interpreter");

    ROS_INFO_STREAM("Starting control interpreter");

    _input_velocity_sub = nh.subscribe("cmd_vel", 1,
        &ControlInterpreterCore::InputVelocityReceived, this);
    _input_speed_sub = nh.subscribe("speed", 1, 
        &ControlInterpreterCore::InputSpeedReceived, this);
    _input_foot_sub = nh.subscribe("foot", 1, 
        &ControlInterpreterCore::InputFootReceived, this);
    _input_height_sub = nh.subscribe("height", 1, 
        &ControlInterpreterCore::InputHeightReceived, this);
    _input_gait_sub = nh.subscribe("gait", 1, 
        &ControlInterpreterCore::InputGaitReceived, this);
    _input_step_sub = nh.subscribe("step", 1, 
        &ControlInterpreterCore::InputStepReceived, this);
    _input_balance_sub = nh.subscribe("balance", 1, 
        &ControlInterpreterCore::InputBalanceReceived, this);

    _is_new_message_sub = nh.subscribe("/NewMessage", 1, 
        &ControlInterpreterCore::SendNewMessage, this);

    _walk_pub = nh.advertise<antdroid_msgs::Walk>("/Walk", 1);
    _rotate_pub = nh.advertise<antdroid_msgs::Rotate>("/Rotate", 1);
    _speed_pub = nh.advertise<antdroid_msgs::Speed>("/Speed", 1);
    _foot_pub = nh.advertise<antdroid_msgs::Foot>("/Foot", 1);
    _height_pub = nh.advertise<antdroid_msgs::Height>("/Height", 1);
    _gait_pub = nh.advertise<antdroid_msgs::Gait>("/Gait", 1);
    _balance_pub = nh.advertise<antdroid_msgs::Balance>("/Balance", 1);


    return true;
}

void ControlInterpreterCore::spin()
{
    ros::Rate loop_rate(SPIN_FRECUENCY);


    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        CheckNewMessageCounter();
    }

}

void ControlInterpreterCore::InputVelocityReceived(
    const geometry_msgs::TwistPtr& input_velocity)
{
    if(_new_message_count == 0)
        return;

    _new_message_count -= 1;

    _walk->x = 0;
    _walk->y = 0;

    _rotate->yaw = 0;

    if(input_velocity->linear.x != 0.0 && input_velocity->linear.y != 0.0  )
    {
        _walk->x = ((int)(input_velocity->linear.x * (_step/1.41)));
        _walk->y = ((int)(input_velocity->linear.y * (_step/1.41)));

        _walk_pub.publish(_walk);
    }

    else if(input_velocity->linear.x != 0.0 )
    {
        _walk->x = ((int)input_velocity->linear.x * _step);
        _walk_pub.publish(_walk);
    }

    else if(input_velocity->linear.y != 0.0 )
    {
        _walk->y = ((int)input_velocity->linear.y * _step);
        _walk_pub.publish(_walk);
    }

    else if(input_velocity->angular.z != 0.0)
    {
        _rotate->yaw = ((int)input_velocity->angular.z * 150);
        _rotate_pub.publish(_rotate);
    }

    else
    {
        ROS_ERROR_STREAM("Coudn't handle move");
    }
}


void ControlInterpreterCore::InputSpeedReceived(
    const antdroid_msgs::Speed& input)
{
    _speed_pub.publish(input);

}

void ControlInterpreterCore::InputFootReceived(
    const antdroid_msgs::Foot& input)
{
    _foot_pub.publish(input);

}

void ControlInterpreterCore::InputHeightReceived(
    const antdroid_msgs::Height& input)
{
    _height_pub.publish(input);

}

void ControlInterpreterCore::InputGaitReceived(
    const antdroid_msgs::Gait& input)
{
    _gait_pub.publish(input);

}

void ControlInterpreterCore::InputStepReceived(
    const std_msgs::Bool& input)
{
    if(input.data && (_step + 3 < 60) )
        _step += 3;
    else if (_step - 3 > 0 && !input.data)
        _step -= 3;
}

void ControlInterpreterCore::InputBalanceReceived(
    const antdroid_msgs::Balance& input)
{
    _balance_pub.publish(input);

}

void ControlInterpreterCore::SendNewMessage(const std_msgs::Bool& msg)
{
    if(msg.data)
        _new_message_count += 1;
    else
        _new_message_count = INIT_NEW_MESSAGE_COUNTER;
}

void ControlInterpreterCore::CheckNewMessageCounter()
{
    if(_new_message_count == 0)
        _checker_count += 1;
    else
        _checker_count = 0;

    if(_checker_count > SPIN_FRECUENCY * SECONDS_UNTIL_RESTART ||
        _new_message_count > INIT_NEW_MESSAGE_COUNTER)
    {
        _checker_count = 0;
        _new_message_count = INIT_NEW_MESSAGE_COUNTER;
    }

}

}
