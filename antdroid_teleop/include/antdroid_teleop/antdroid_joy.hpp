/* antdroid_joy.hpp: header of antdroid_joy.cpp.
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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <antdroid_msgs/Walk.h>
#include <antdroid_msgs/Rotate.h>
#include <antdroid_msgs/Speed.h>
#include <antdroid_msgs/Foot.h>
#include <antdroid_msgs/Height.h>
#include <antdroid_msgs/Gait.h>
#include <antdroid_msgs/Balance.h>


/***** PS3 BUTTONS AND AXES DEFINES ********************************/

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
//#define PS3_AXIS_ACCELEROMETER_LEFT      16
//#define PS3_AXIS_ACCELEROMETER_FORWARD   17
//#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

#define PS3_AXIS_ACCELEROMETER_LEFT      23
#define PS3_AXIS_ACCELEROMETER_FORWARD   24
#define PS3_AXIS_ACCELEROMETER_UP        25


/*******************************************************************/

#define DECREASE_SPEED      0
#define RISE_SPEED          1

#define DECREASE_FOOT       0
#define RISE_FOOT           1

#define DECREASE_HEIGHT     0
#define RISE_HEIGHT         1

#define DECREASE_STEP       0
#define RISE_STEP           1

#define ROTATE_LEFT         1
#define ROTATE_RIGHT        -1

#define TRIPOD_MODE 1
#define RIPPLE_MODE 2

#define MAX_ANGLE       180
#define ANGLE_STEP      20
#define DEAD_ZONE       0.5


class AntdroidTeleop
{
public:
    AntdroidTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void publish();

    void manageBalance(void);
    int  updateAngle(int max_angle, int axis, int last_angle);

    ros::NodeHandle _ph, _nh;

    int _rotate_right;
    int _rotate_left;

    int _rise_speed;
    int _decrease_speed;

    int _rise_height;
    int _decrease_height;

    int _rise_foot;
    int _decrease_foot;

    int _rise_step;
    int _decrease_step;

    int _change_gait;

    int _rise_sens_accel;
    int _decrease_sens_accel;
    
    int _walk_x;
    int _walk_y;

    int _balance_x;
    int _balance_y;
    int _balance_z;
    int _balance_default;

    int _balance_accel_pitch;
    int _balance_accel_roll;
    int _balance_gyro_yaw;

    int _action_button;
    int _balance_mode;

    int _attack;
    int _say_hello;

    float _dead_zone_accel;

    bool _new_balance_msg;
    bool _new_balance_z_msg;
    bool _new_gait_msg;
    bool _new_height_msg;
    bool _new_foot_msg;
    bool _new_rotate_msg;
    bool _new_speed_msg;
    bool _new_vel_msg;
    bool _new_step_msg;
    bool _new_balance_default_msg;
    bool _new_attack_msg;
    bool _new_say_hello_msg;
    bool _new_balance_accel_msg;

    int _last_pitch;
    int _pitch;
    int _last_roll;
    int _roll;
    int _last_yaw;
    int _yaw;

    int _gait_type;    

    ros::Subscriber sub_joy;

    ros::Publisher  _pub_balance;
    ros::Publisher  _pub_gait;
    ros::Publisher  _pub_height;
    ros::Publisher  _pub_foot;
    ros::Publisher  _pub_speed;
    ros::Publisher  _pub_step;
    ros::Publisher  _pub_vel;
    ros::Publisher  _pub_attack;
    ros::Publisher  _pub_say_hello;

    antdroid_msgs::Balance      _msg_balance;
    antdroid_msgs::Gait         _msg_gait;
    antdroid_msgs::Height       _msg_height;
    antdroid_msgs::Foot         _msg_foot;
    antdroid_msgs::Speed        _msg_changeSpeed;
    geometry_msgs::Twist        _msg_vel;
    std_msgs::Bool              _msg_step;
    std_msgs::Bool              _msg_attack;
    std_msgs::Bool              _msg_say_hello;
    
    boost::mutex _publish_mutex;
    ros::Timer _timer;
};

int Abs(int value);