/* control_interpreter_core.hpp: header of control_interpreter_core.
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

#ifndef CONTROL_INTERPRETER_CORE_HPP_
#define CONTROL_INTERPRETER_CORE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <antdroid_msgs/Walk.h>
#include <antdroid_msgs/Rotate.h>
#include <antdroid_msgs/Speed.h>
#include <antdroid_msgs/Foot.h>
#include <antdroid_msgs/Height.h>
#include <antdroid_msgs/Gait.h>
#include <antdroid_msgs/Balance.h>

static const int SPIN_FRECUENCY = 8;
static const int INIT_NEW_MESSAGE_COUNTER = 2;
static const int SECONDS_UNTIL_RESTART = 3;

namespace control_interpreter_core
{
    class ControlInterpreterCore
    {
    public:
        ControlInterpreterCore();
        ~ControlInterpreterCore();
        bool init();

        void spin();

    private:
        ros::Publisher _walk_pub;
        ros::Publisher _rotate_pub;
        ros::Publisher _speed_pub;
        ros::Publisher _foot_pub;
        ros::Publisher _height_pub;
        ros::Publisher _gait_pub;
        ros::Publisher _balance_pub;


        int _step;
        int _new_message_count;
        int _checker_count;

        antdroid_msgs::WalkPtr _walk;
        antdroid_msgs::RotatePtr _rotate;

        ros::Subscriber _input_velocity_sub;
        ros::Subscriber _input_speed_sub;
        ros::Subscriber _input_foot_sub;
        ros::Subscriber _input_height_sub;
        ros::Subscriber _input_gait_sub;
        ros::Subscriber _input_step_sub;
        ros::Subscriber _input_balance_sub;

        ros::Subscriber _is_new_message_sub;


        void InputVelocityReceived(
            const geometry_msgs::TwistPtr& input_velocity);
        void InputSpeedReceived(const antdroid_msgs::Speed& input);
        void InputFootReceived(const antdroid_msgs::Foot& input);
        void InputHeightReceived(const antdroid_msgs::Height& input);
        void InputGaitReceived(const antdroid_msgs::Gait& input);
        void InputStepReceived(const std_msgs::Bool& input);
        void InputBalanceReceived(const antdroid_msgs::Balance& input);
        
        void CheckNewMessageCounter();
        void SendNewMessage(const std_msgs::Bool& msg);

    };
}


#endif