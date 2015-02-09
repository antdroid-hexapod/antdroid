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

    _rotate_right           (PS3_BUTTON_REAR_RIGHT_1),
    _rotate_left            (PS3_BUTTON_REAR_LEFT_1),

    _rise_speed             (PS3_BUTTON_CROSS_UP),
    _decrease_speed         (PS3_BUTTON_CROSS_DOWN),

    _rise_height            (PS3_BUTTON_CROSS_RIGHT),
    _decrease_height        (PS3_BUTTON_CROSS_LEFT),

    _rise_foot              (PS3_BUTTON_CROSS_UP),
    _decrease_foot          (PS3_BUTTON_CROSS_DOWN),

    _rise_step              (PS3_BUTTON_CROSS_RIGHT),
    _decrease_step          (PS3_BUTTON_CROSS_LEFT),

    _walk_y                 (PS3_AXIS_STICK_LEFT_LEFTWARDS),
    _walk_x                 (PS3_AXIS_STICK_LEFT_UPWARDS),
    _change_gait            (PS3_BUTTON_STICK_LEFT),

    _balance_x              (PS3_AXIS_STICK_RIGHT_UPWARDS),
    _balance_y              (PS3_AXIS_STICK_RIGHT_LEFTWARDS),
    _balance_z              (PS3_AXIS_STICK_RIGHT_LEFTWARDS),
    _balance_default        (PS3_BUTTON_STICK_RIGHT),

    _attack                 (PS3_BUTTON_ACTION_TRIANGLE),
    _engage                 (PS3_BUTTON_START),
    _disengage              (PS3_BUTTON_SELECT),

    _action_button          (PS3_BUTTON_REAR_RIGHT_2),
    _balance_mode           (PS3_BUTTON_REAR_LEFT_2),

    _balance_accel_pitch    (PS3_AXIS_ACCELEROMETER_FORWARD_COMP),
    _balance_accel_roll     (PS3_AXIS_ACCELEROMETER_LEFT_COMP),
    _balance_gyro_yaw       (PS3_AXIS_ACCELEROMETER_UP_COMP),

    _last_pitch             (0),
    _last_roll              (0),
    _last_yaw               (0),

    _gait_type              (1),

    _new_balance_msg        (false),
    _new_balance_z_msg      (false),
    _new_gait_msg           (false),
    _new_height_msg         (false),
    _new_foot_msg           (false),
    _new_rotate_msg         (false),
    _new_speed_msg          (false),
    _new_vel_msg            (false),
    _new_step_msg           (false),
    _new_balance_default_msg(false),
    _new_attack_msg         (false),
    _new_engage_msg         (false),
    _new_disengage_msg      (false),
    _new_balance_accel_msg  (false)
{      
 
    sub_joy = _nh.subscribe<sensor_msgs::Joy>("joy", 1, &AntdroidTeleop::joyCallback, this);
    
    _pub_vel         = _ph.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _pub_balance     = _ph.advertise<antdroid_msgs::Balance>("balance", 1);
    _pub_gait        = _ph.advertise<antdroid_msgs::Gait>("gait", 1);
    _pub_height      = _ph.advertise<antdroid_msgs::Height>("height", 1);
    _pub_foot        = _ph.advertise<antdroid_msgs::Foot>("foot", 1);
    _pub_speed       = _ph.advertise<antdroid_msgs::Speed>("speed", 1);
    _pub_step        = _ph.advertise<std_msgs::Bool>("step", 1);

    _timer = _nh.createTimer(ros::Duration(0.1), boost::bind(&AntdroidTeleop::publish, this));
}

void AntdroidTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
    /*****************  SPEED ********* CROSS PAD *****************************/    
    if(!joy->buttons[_action_button] &&
      (joy->buttons[_rise_speed] || joy->buttons[_decrease_speed]))
    {
        if(joy->buttons[_rise_speed])
            _msg_changeSpeed.speed = RISE_SPEED;
        if(joy->buttons[_decrease_speed])
            _msg_changeSpeed.speed = DECREASE_SPEED;

        _new_speed_msg = true;
    }

    /*****************  HEIGHT ********* CROSS PAD *****************************/ 
    if(!joy->buttons[_action_button] &&
      (joy->buttons[_rise_height] || joy->buttons[_decrease_height]))
    {
        if(joy->buttons[_rise_height])
            _msg_height.height = RISE_HEIGHT;
        if(joy->buttons[_decrease_height])
            _msg_height.height = DECREASE_HEIGHT;
        
        _new_height_msg = true;
    }

    /*****************  FOOT ********* CROSS PAD + R2 ************************/ 
    if(joy->buttons[_action_button] && 
      (joy->buttons[_rise_foot] || joy->buttons[_decrease_foot]))
    {
        if(joy->buttons[_rise_foot])
            _msg_foot.footDistance = RISE_FOOT;
        if(joy->buttons[_decrease_foot])
            _msg_foot.footDistance = DECREASE_FOOT;

        _new_foot_msg = true;
    }

    /*****************  STEP ********* CROSS PAD + R2 ************************/ 
    if(joy->buttons[_action_button] && 
      (joy->buttons[_rise_step] || joy->buttons[_decrease_step]))
    {
        if(joy->buttons[_rise_step])
            _msg_step.data = true;
        if(joy->buttons[_decrease_step])
            _msg_step.data = false;

        _new_step_msg = true;
    }

    /*****************  GAIT **************************************************/
    if(joy->buttons[_change_gait])
    {
        if(_gait_type == TRIPOD_MODE)
            _gait_type = RIPPLE_MODE;  //type [1, 2] -> [tripod, riple]
        else
            _gait_type = TRIPOD_MODE;

        _msg_gait.type = _gait_type;

        _new_gait_msg = true;
    }

    /*****************  WALK *************************************************/
    if(joy->axes[_walk_x] || joy->axes[_walk_y])
    {
        _msg_vel.linear.x = joy->axes[_walk_x];
            if(_msg_vel.linear.x > DEAD_ZONE)         _msg_vel.linear.x = 1;
            else if(_msg_vel.linear.x < -DEAD_ZONE)   _msg_vel.linear.x = -1;
            else                                      _msg_vel.linear.x = 0;

        _msg_vel.linear.y = joy->axes[_walk_y];
            if(_msg_vel.linear.y > DEAD_ZONE)         _msg_vel.linear.y = 1;
            else if(_msg_vel.linear.y < -DEAD_ZONE)   _msg_vel.linear.y = -1;
            else                                      _msg_vel.linear.y = 0;

        _msg_vel.angular.z = 0;

        /*ROS_INFO("_msg_vel(linear.x, linear.y, angular.z) =  ( %f, %f, %f) ",
            _msg_vel.linear.x,_msg_vel.linear.y,_msg_vel.angular.z);*/

        _new_vel_msg = true;
    }
    
    /*****************  ROTATE ************************************************/
    if(joy->buttons[_rotate_left] && !joy->buttons[_rotate_right])
    {
        _msg_vel.linear.x = 0;
        _msg_vel.linear.y = 0;
        _msg_vel.angular.z = ROTATE_LEFT;

        _new_rotate_msg = true;
    }

    if(joy->buttons[_rotate_right] && !joy->buttons[_rotate_left])
    {
        _msg_vel.linear.x = 0;
        _msg_vel.linear.y = 0;
        _msg_vel.angular.z = ROTATE_RIGHT;

        _new_rotate_msg = true;
    }

    /*****************  BALANCE JOYSTICK***************************************/
    if(!joy->buttons[_action_button] &&
      (joy->axes[_balance_x] || joy->axes[_balance_y]))
    {
        _pitch = joy->axes[_balance_x];
            if(_pitch > DEAD_ZONE)         _pitch = 1;
            else if(_pitch < - DEAD_ZONE)  _pitch = -1;
            else                           _pitch = 0;
        
        _roll = joy->axes[_balance_y];
            if(_roll > DEAD_ZONE)         _roll = 1;
            else if(_roll < -DEAD_ZONE)   _roll = -1;
            else                          _roll = 0;

        _yaw = 0;

        _new_balance_msg = true;
    }

    if(joy->buttons[_action_button] && (joy->axes[_balance_z]))
    {
        _msg_balance.pitch = 0;
        _msg_balance.roll = 0 ;

        _last_pitch = 0;
        _last_roll = 0;

        _yaw = joy->axes[_balance_z];
            if(_yaw > DEAD_ZONE)        _yaw = 1;
            else if(_yaw < - DEAD_ZONE) _yaw = -1;
            else                        _yaw = 0;

        _new_balance_z_msg = true;
    }

    if(joy->buttons[_balance_default])
    {
        _msg_balance.pitch = 0;
        _msg_balance.roll = 0;
        _msg_balance.yaw = 0;

        _last_pitch = 0;
        _last_roll = 0;
        _last_yaw = 0;

        _new_balance_default_msg = true;
    }

    /*****************  BALANCE ACCELEROMETERS ********************************/
    if(joy->buttons[_balance_mode]  && (
        joy->axes[_balance_accel_pitch] ||
        joy->axes[_balance_accel_roll]))
    {
            if(joy->axes[_balance_accel_pitch] < - DEAD_ZONE_ACCEL)
                _pitch = 1;
            else if(joy->axes[_balance_accel_pitch] > DEAD_ZONE_ACCEL)
                _pitch = -1;
            else
                _pitch = 0;
        
            if(joy->axes[_balance_accel_roll] < - DEAD_ZONE_ACCEL)
                _roll = 1;
            else if(joy->axes[_balance_accel_roll] > DEAD_ZONE_ACCEL)
                _roll = -1;
            else
                _roll = 0;

        _yaw = 0;

        ROS_INFO("pitch, roll, yaw: %d, %d, %d", _pitch, _roll, _yaw);
        _new_balance_accel_msg = true;
    }

    if(joy->buttons[_balance_mode] && (
        !joy->axes[_balance_accel_pitch] &&
        !joy->axes[_balance_accel_roll]))
    {
        _msg_balance.pitch = 0;
        _msg_balance.roll = 0;
        _msg_balance.yaw = 0;

        _last_pitch = 0;
        _last_roll = 0;
        _last_yaw = 0;

        _new_balance_default_msg = true;
    }

    /*****************  ATTACK ************************************************/
    /*if(joy->buttons[_attack])
    {

        _new_attack_msg = true;
    }

    /*****************  ENGAGE ************************************************
    if(joy->buttons[_engage])
    {

        _new_engage_msg = true;
    }

    /*****************  DISENGAGE *********************************************
    if(joy->buttons[_disengage])
    {

        _new_disengage_msg = true;
    }*/
}

void AntdroidTeleop::publish()
{
    boost::mutex::scoped_lock lock(_publish_mutex);

    if(_new_gait_msg)
    {
        ROS_INFO_STREAM("publish:: gait");
        _pub_gait.publish(_msg_gait);
        _new_gait_msg = false;
    }

    if(_new_height_msg)
    {
        ROS_INFO_STREAM("publish:: height");
        _pub_height.publish(_msg_height);
        _new_height_msg = false;
    }

    if(_new_foot_msg)
    {
        ROS_INFO_STREAM("publish:: foot");
        _pub_foot.publish(_msg_foot);
        _new_foot_msg = false;
    }

    if(_new_speed_msg)
    {
        ROS_INFO_STREAM("publish:: speed");
        _pub_speed.publish(_msg_changeSpeed);
        _new_speed_msg = false;
    }

    if(_new_step_msg)
    {
        ROS_INFO_STREAM("publish:: step");
        _pub_step.publish(_msg_step);
        _new_step_msg = false;
    }

    if(_new_vel_msg)
    {
        ROS_INFO_STREAM("publish:: walk");
        _pub_vel.publish(_msg_vel);
        _new_vel_msg = false;
    }

    if(_new_rotate_msg)
    {
        ROS_INFO_STREAM("publish:: rotate");
        _pub_vel.publish(_msg_vel);
        _new_rotate_msg = false;
    }

    if(_new_balance_msg)
    {
        ROS_INFO_STREAM("publish:: balance");
        
        manageBalance();
        _pub_balance.publish(_msg_balance);
        _new_balance_msg = false;
    }

    if(_new_balance_z_msg)
    {
        ROS_INFO_STREAM("publish:: balance z");
        
        manageBalance();
        _pub_balance.publish(_msg_balance);
        _new_balance_z_msg = false;
    }

    if(_new_balance_default_msg)
    {
        ROS_INFO_STREAM("publish:: balance default");

        _pub_balance.publish(_msg_balance);
        _new_balance_default_msg = false;
    }

    if(_new_balance_accel_msg)
    {
        ROS_INFO_STREAM("publish:: balance accel");
        
        manageBalance();
        _pub_balance.publish(_msg_balance);
        _new_balance_accel_msg = false;
    }
/*
    if(_new_attack_msg)
    {
        ROS_INFO_STREAM("publish:: attack");
        _new_attack_msg = false;
    }

    if(_new_engage_msg)
    {
        ROS_INFO_STREAM("publish:: engage");
        _new_engage_msg = false;
    }

    if(_new_disengage_msg)
    {
        ROS_INFO_STREAM("publish:: disengage");
        _new_disengage_msg = false;
    }*/
}

void AntdroidTeleop::manageBalance()
{
    _last_pitch = updateAngle(_pitch, _last_pitch);
    _last_roll = updateAngle(_roll, _last_roll);
    _last_yaw = updateAngle(_yaw, _last_yaw);

    _msg_balance.pitch = _last_pitch;
    _msg_balance.roll = _last_roll;
    _msg_balance.yaw = _last_yaw;
}

int AntdroidTeleop::updateAngle(int axis, int last_angle)
{
    _max_angle_step = ANGLE_STEP * 5;

    if ((last_angle + axis * ANGLE_STEP > (- _max_angle_step)) & 
        (last_angle + axis * ANGLE_STEP < _max_angle_step) & (axis != 0))
    {
        last_angle += axis * ANGLE_STEP;
    }

    return(last_angle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "antdroid_teleop");
    AntdroidTeleop antdroid_teleop;

    ros::spin();
}