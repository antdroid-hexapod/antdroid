/* log.cpp: generic log functions 
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
* A generic set of log functions divided hierarchically by its 
* relevance: error, warn, info anf bebug messages. The higher the
* value of the 'level_log' global parameter, the higher number of 
* functions we turn on. 'level_log=1' means only error messages turned
* on while 'level_log=3' means error, warn anf info messages turned on
*
* The functions:
*
* log(String, int): sends by serial port a message. The int parameter sets
*                   the level of the message we send.
* log(String, int, int): log overloaded. Allows us to send the value of
*                   one parameter to the printDebug function.
*
* printMsg(String, int): sends by serial port a message if level_log = [1-4]
* printMsg(String, int, int): printMsg overloaded. Allows us to send a
*                     message and a parameter if level_log >=4
*/

#include "log.h"

void log(String msg, int code)
{
    if(level_log >= code)
        printMsg(msg, code);
}

void log(String msg, int attribute, int code)
{
    if(level_log >= code)
        printMsg(msg, attribute, code);
}

#ifdef ControlSerial
    void printMsg(String msg, int code)
    {
        switch(code)
        {
            case 1:
                Serial.print("Error: ");
                break;
            case 2:
                Serial.print("Warn: ");
                break;
            case 3:
                Serial.print("Info: ");
                break;
            case 4:
                Serial.print("Debug: ");
                break;
            default:
                Serial.print("log.cpp. printMsg.");
                break;
        }
        Serial.println(msg);
    }

    void printMsg(String msg, int attribute,int code)
    {
        switch(code)
        {
            case 1:
                Serial.print("Error: ");
                break;
            case 2:
                Serial.print("Warn: ");
                break;
            case 3:
                Serial.print("Info: ");
                break;
            case 4:
                Serial.print("Debug: ");
                break;
            default:
                Serial.print("log.cpp. printMsg.");
                break;
        }
        Serial.print(msg);
        Serial.print(" = ");
        Serial.println(attribute);
    }

#else

    #ifdef ControlRos

        void printMsg(String msg, int code)
        {
            char* charMsg = StringToChar(msg);
            switch(code)
            {
                case 1:
                    arduino.logerror(charMsg);
                    break;
                case 2:
                    arduino.logwarn(charMsg);
                    break;
                case 3:
                    arduino.loginfo(charMsg);
                    break;
                case 4:
                    arduino.logdebug(charMsg);
                    break;
                default:
                    arduino.logerror("log.cpp. printMsg.");
                    break;
            }
            delete charMsg;
        }

        void printMsg(String msg, int attribute, int code)
        {
            char* charMsg = StringToChar(msg);
            String stringAttribute = String(attribute);
            char* charAtrribute = StringToChar(stringAttribute);

            switch(code)
            {
                case 1:
                    arduino.logerror(charMsg);
                    arduino.logerror(charAtrribute);
                    break;
                case 2:
                    arduino.logwarn(charMsg);
                    arduino.logwarn(charAtrribute);
                    break;
                case 3:
                    arduino.loginfo(charMsg);
                    arduino.loginfo(charAtrribute);                
                    break;
                case 4:
                    arduino.logdebug(charMsg);
                    arduino.logdebug(charAtrribute);
                    break;
                default:
                    arduino.logerror("log.cpp. printMsg.");
                    break;
            }

            delete charMsg;
            delete charAtrribute;
        }

    #else
        void printMsg(String msg, int code){}
        void printMsg(String msg, int attribute, int code){}
    #endif
#endif

inline char* StringToChar(String msg)
{
    char* charBuff = new char[msg.length()+1];
    msg.toCharArray(charBuff, msg.length()+1);
    return charBuff;
}