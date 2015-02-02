/* log.h: generic log functions 
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

#ifndef log_h
#define log_h

#include "Arduino.h"

#include "Configuration.h"

#define Debug   4
#define Info    3
#define Warn    2
#define Error   1
 
extern byte level_log;

#ifdef ControlRos
    #include "ros.h"
    extern ros::NodeHandle arduino;
#endif
    
void log(String msg, int code);
void log(String msg, int attribute, int code);

void printMsg(String msg, int code);
void printMsg(String msg, int attribute, int code);

char* StringToChar(String msg);

#endif