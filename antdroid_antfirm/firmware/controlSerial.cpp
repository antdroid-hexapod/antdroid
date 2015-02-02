/* controlSerial.cpp: serial control for hexapods
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
  * board. Warn: It is not compatible with ROS control system.
  *
  *The methos are:
  *
  * Control: Class for manipulating a hexapod by using the serial port
  * Start(): begins the serial communication (baud rate:115200)
  * ReadInput(): reads the commands introduced by the user by serial port
  * 
  * RunCommand(): main menu for the rest of the methods.
  * TryRunMove(byte ): moves the hexapod to the selected (X, Y) coordinate.
  * TryRunTurn(byte ): spins the hexapod the selected number of degrees.
  * TryRunGait(byte ): selects the gait that the hexapod is going to use.
  * TryRunBalance(byte ): balances the hexapod
  * TryRunSpeed(byte ): changes the movement speed of the hexapod [0 - 250]
  * TryRunHeight(byte ): changes the current height of the hexapod.
  * TryRunFootDistance(byte ): changes foot distance of hexapod's legs
  * TryChangeLog(byte ): changes the amount of info given by the log
  * TryChangeCalibration(byte ): runs calibration mode.
  */

#include "Configuration.h"

#ifdef ControlSerial
#include "controlSerial.h"

Control::Control(Hexapod* Antdroid)
{
    log("In Control::Control", Debug);

    _antdroid = Antdroid;
    _command = "";
}

void Control::Start(void)
{
    level_log = 3;

    Serial.begin(115200);
    log("Serial.begin = 115200", Info);
}

void Control::ReadInput(void)
{
    log("In Control::ReadInput", Debug);

    while(Serial.available() >0 )
    {
        _current_char = Serial.read();
        if(_current_char == '\n' ||
            _current_char == '\r'||
            _current_char == ';'
            )
        {
           RunCommand();
           _command = ""; 
        }
        else
        {
            _command += _current_char; 
        }
    }

}

void Control::RunCommand(void)
{
    log("In Control::RunCommand", Debug);

    byte endCommandName = 0;

    while(endCommandName < _command.length() &&
    _command.charAt(endCommandName) != '(' &&
    _command.charAt(endCommandName) != ' ')
    {
        endCommandName++;
        
    }

    if(_command.substring(0, endCommandName) == "move")
    {
        if(!TryRunMove(endCommandName))
            RunDefault();    
    }
    else if (_command.substring(0, endCommandName) == "turn")
    {
        if(!TryRunTurn(endCommandName))
            RunDefault();
    }
    else if (_command.substring(0, endCommandName) == "gait")
    {
        if(!TryRunGait(endCommandName))
            RunDefault();
    }
    else if (_command.substring(0, endCommandName) == "balance")
    {
        if(!TryRunBalance(endCommandName))
            RunDefault();
    }
    else if (_command.substring(0, endCommandName) == "speed")
    {
        if(!TryRunSpeed(endCommandName))
            RunDefault();
    }
    else if (_command.substring(0, endCommandName) == "height")
    {
        if(!TryRunHeight(endCommandName))
            RunDefault();
    }
    else if (_command.substring(0, endCommandName) == "foot_distance")
    {
        if(!TryRunFootDistance(endCommandName))
            RunDefault();
    }
     else if (_command.substring(0, endCommandName) == "calibrate")
    {
        if(!TryChangeCalibration(endCommandName))
            RunDefault();
    }
    else if (_command.substring(0, endCommandName) == "log")
    {
        if(!TryChangeLog(endCommandName))
            RunDefault();
    }
    else if (_command == "help")
        RunHelp();
    else
        RunDefault();

}


bool Control::TryRunMove(byte endCommandName)
{
    log("In Control::TryRunMove", Debug);

    String number;

    byte initString;
    byte endString;
    short x;
    short y;

    endString = endCommandName;


    if(!_command.startsWith("(", endString))
        return false;

    initString = ++endString;

    while(endString < _command.length() &&
    _command.charAt(endString) != ',' )
    {
        endString++;
    }

    if(!_command.startsWith(",", endString))
        return false;

    number = _command.substring(initString, endString);

    if( number == "0")
        x = 0;
    else if(number.toInt() != 0)
        x = number.toInt();
    else
        return false;

    initString = ++endString;


    while(endString < _command.length() &&
    _command.charAt(endString) != ')' )
    {
        endString++;
    }

    if(!_command.startsWith(")", endString))
        return false;

   number = _command.substring(initString, endString);

    if( number == "0")
        y = 0;
    else if(number.toInt() != 0)
        y = number.toInt();
    else
        return false;

    _antdroid->Walk(x,y);

    return true;
}



bool Control::TryRunTurn(byte endCommandName)
{
    log("In Control::TryRunTurn", Debug);

    String number;

    byte initString;
    byte endString;
    short degree;

    endString = endCommandName;


    if(!_command.startsWith("(", endString))
        return false;

    initString = ++endString;

    while(endString < _command.length() &&
    _command.charAt(endString) != ')' )
    {
        endString++;
    }

    if(!_command.startsWith(")", endString))
        return false;

    number = _command.substring(initString, endString);

    if( number == "0")
        degree = 0;
    else if(number.toInt() != 0)
        degree = number.toInt();
    else
        return false;

    _antdroid->Rotate(degree);
    return true;
}

bool Control::TryRunGait(byte endCommandName)
{
    log("In Control::TryRunGait", Debug);

    byte initString;
    byte endString;

    endString = endCommandName;

    while(endString < _command.length() &&
    _command.charAt(endString) != '-' )
    {
        endString++;
    }

    if(!_command.startsWith("-", endString))
        return false;

    initString = ++endString;

    if(_command.startsWith("-", endString))
    {
        initString = ++endString;
        
        while(endString < _command.length() &&
        _command.charAt(endString) != ' ' )
        {
            endString++;
        }

        if(_command.substring(initString, endString) == "tripod")
        {
            _antdroid->EnableTripodGait();
            return true;
        }
        if(_command.substring(initString, endString) == "ripple")
        {
            _antdroid->EnableRippleGait();
            return true;
        }
        else
            return false;
    }

    else if (_command.charAt(endString) == 't' && endString == _command.length()-1)
    {
        _antdroid->EnableTripodGait();
        return true;
    }

    else if (_command.charAt(endString) == 'r' && endString == _command.length()-1)
    {
        _antdroid->EnableRippleGait();
        return true;
    }


    return false;

}
bool Control::TryRunBalance(byte endCommandName)
{
    log("In Control::TryRunBalance", Debug);

    String number;

    byte initString;
    byte endString;
    short pitch;
    short roll;
    short yaw;

    initString = endCommandName;


    if(!_command.startsWith("(", initString))
        return false;

    initString++;

    endString = initString; 

    while(endString < _command.length() &&
    _command.charAt(endString) != ',' )
    {
        endString++;
    }

    if(!_command.startsWith(",", endString))
        return false;

    number = _command.substring(initString, endString);

    if( number == "0")
        pitch = 0;
    else if(number.toInt() != 0)
        pitch = number.toInt();
    else
        return false;

    initString = ++endString;

    while(endString < _command.length() &&
    _command.charAt(endString) != ',' )
    {
        endString++;
    }

    if(!_command.startsWith(",", endString))
        return false;

    number = _command.substring(initString, endString);

    if( number == "0")
        roll = 0;
    else if(number.toInt() != 0)
        roll = number.toInt();
    else
        return false;

    initString = ++endString;

    while(endString < _command.length() &&
    _command.charAt(endString) != ')' )
    {
        endString++;
    }

    if(!_command.startsWith(")", endString))
        return false;

   number = _command.substring(initString, endString);

    if( number == "0")
        yaw = 0;
    else if(number.toInt() != 0)
        yaw = number.toInt();
    else
        return false;

    _antdroid->Balance(pitch, roll, yaw);

    return true;
}

bool Control::TryRunSpeed(byte endCommandName)
{  
    log("In Control::TryRunSpeed", Debug);

    String number;

    byte initString;
    byte endString;

    endString = endCommandName;

    while(endString < _command.length() &&
    _command.charAt(endString) != '-' )
    {
        endString++;
    }

    if(!_command.startsWith("-", endString))
        return false;

    initString = ++endString;

    if (_command.charAt(endString) == 'r')
    {
        _antdroid->RiseSpeed();
        return true;
    }

    else if(_command.charAt(endString) == 'd')
    {
        _antdroid->DecreaseSpeed();
        return true;
    }

    else if(_command.charAt(endString) == 'c')
    {
        initString = ++endString;
        
        while(endString < _command.length() &&
        _command.charAt(endString) != ' ' )
        {
            endString++;
        }
        
        number = _command.substring(initString, endString);

        if(number.toInt() != 0)
        {
            _antdroid->ChangeSpeedStep(number.toInt());
            return true; 
        }
    }
    return false;
}

bool Control::TryRunHeight(byte endCommandName)
{  
    log("In Control::TryRunHeight", Debug);

    String number;

    byte initString;
    byte endString;

    endString = endCommandName;

    while(endString < _command.length() &&
    _command.charAt(endString) != '-' )
    {
        endString++;
    }

    if(!_command.startsWith("-", endString))
        return false;

    initString = ++endString;

    if (_command.charAt(endString) == 'r')
    {
        _antdroid->RiseHeight();
        return true;
    }

    else if(_command.charAt(endString) == 'd')
    {
        _antdroid->DecreaseHeight();
        return true;
    }

    else if(_command.charAt(endString) == 'c')
    {
        initString = ++endString;
        
        while(endString < _command.length() &&
        _command.charAt(endString) != ' ' )
        {
            endString++;
        }
        
        number = _command.substring(initString, endString);

        if(number.toInt() != 0)
        {
            _antdroid->ChangeHeightStep(number.toInt());
            return true; 
        }
    }
    return false;
}

bool Control::TryRunFootDistance(byte endCommandName)
{  
    log("In Control::TryRunDistance", Debug);

    String number;

    byte initString;
    byte endString;

    endString = endCommandName;

    while(endString < _command.length() &&
    _command.charAt(endString) != '-' )
    {
        endString++;
    }

    if(!_command.startsWith("-", endString))
        return false;

    initString = ++endString;

    if (_command.charAt(endString) == 'r')
    {
        _antdroid->RiseFootDistance();
        return true;
    }

    else if(_command.charAt(endString) == 'd')
    {
        _antdroid->DecreaseFootDistance();
        return true;
    }

    else if(_command.charAt(endString) == 'c')
    {
        initString = ++endString;
        
        while(endString < _command.length() &&
        _command.charAt(endString) != ' ' )
        {
            endString++;
        }
        
        number = _command.substring(initString, endString);

        if(number.toInt() != 0)
        {
            _antdroid->ChangeFootDistanceStep(number.toInt());
            return true; 
        }
    }
    return false;
}

bool Control::TryChangeLog(byte endCommandName)
{
    log("In Control::TryChangeLog", Debug);

    String number;

    byte initString;
    byte endString;

    endString = endCommandName;

    while(endString < _command.length() &&
    _command.charAt(endString) != '-' )
    {
        endString++;
    }

    if(!_command.startsWith("-", endString))
        return false;

    initString = ++endString;

    if (_command.charAt(endString) == '1')
    {
        level_log = 1;
        Serial.println("Error messages activated");
        return true;
    }
    else if(_command.charAt(endString) == '2')
    {
        level_log = 2;
        Serial.println("Errors and warns messages activated");
        return true;
    }
    else if(_command.charAt(endString) == '3')
    {
        level_log = 3;
        Serial.println("Errors, warns and info messages activated");
        return true;
    }
    else if(_command.charAt(endString) == '4')
    {
        level_log = 4;
        Serial.println("Errors, warns, info and debug messages activated");
        return true;
    }
    return false;
}

bool Control::TryChangeCalibration(byte endCommandName)
{
    log("In Control::TryChangeCalibration", Debug);

    String number;

    byte initString;
    byte endString;
    short leg;
    short member;
    short angle;

    initString = endCommandName;


    if(!_command.startsWith("(", initString))
        return false;

    initString++;

    endString = initString; 

    while(endString < _command.length() &&
    _command.charAt(endString) != ',' )
    {
        endString++;
    }

    if(!_command.startsWith(",", endString))
        return false;

    number = _command.substring(initString, endString);

    if( number == "0")
        leg = 0;
    else if(number.toInt() != 0)
        leg = number.toInt();
    else
        return false;

    if(leg > 5 || leg < 0)
        return false;

    initString = ++endString;

    while(endString < _command.length() &&
    _command.charAt(endString) != ',' )
    {
        endString++;
    }

    if(!_command.startsWith(",", endString))
        return false;

    number = _command.substring(initString, endString);

    if( number == "0")
        member = 0;
    else if(number.toInt() != 0)
        member = number.toInt();
    else
        return false;

    initString = ++endString;

    while(endString < _command.length() &&
    _command.charAt(endString) != ')' )
    {
        endString++;
    }

    if(!_command.startsWith(")", endString))
        return false;

   number = _command.substring(initString, endString);

    if( number == "0")
        angle = 0;
    else if(number.toInt() != 0)
        angle = number.toInt();
    else
        return false;

    _antdroid->CalibrateLeg(leg, member, angle);

    return true;
}

void Control::RunHelp(void)
{
    Serial.println("Command list:");
    RunHelpMove();
    RunHelpTurn();
    RunHelpGait();
    RunHelpBalance();
    RunHelpSpeed();
    RunHelpHeight();
    RunHelpFootDistance();
    RunHelpCalibration();
    RunHelpLog();
}

void Control::RunHelpMove(void)
{
    Serial.println("move(X,Y)");
    Serial.println("Move hexapod with (X,Y) step with selected gait.");
    Serial.println("X: distance forward in milimeters.");
    Serial.println("Y: distance left side in milimeters.");
    Serial.println(DefaultGaitMsg);
    Serial.println("Example of use: move(30,-20);");
    Serial.println();
}

void Control::RunHelpTurn(void)
{
    Serial.println("turn(Angle)");
    Serial.println("Turn hexapod with selected gait.(positive value: clockwork side)");
    Serial.println("Angle: degrees with one decimal.");
    Serial.println(DefaultGaitMsg);
    Serial.println("Example of use: turn(-150);");
    Serial.println();
}

void Control::RunHelpGait(void)
{
    Serial.println("gait [OPTIONS]");
    Serial.println("Select gait.");
    Serial.println("-t, --tripod");
    Serial.println("    select tripod gait.");
    Serial.println("-r, --ripple");
    Serial.println("    select ripple gait.");
    Serial.println(DefaultGaitMsg);
    Serial.println("Example of use: gait -t;");
    Serial.println();
}

void Control::RunHelpBalance(void)
{
    Serial.println("balance(pitch, roll, yaw)");
    Serial.println("Balance hexapod mode without move.");
    Serial.println("Pitch: pitch angle in degrees with one decimal.");
    Serial.println("Roll: pitch angle in degrees with one decimal.");
    Serial.println("Yaw: pitch angle in degrees with one decimal.");
    Serial.println("Example of use: balance(50,-60,150); ");
    Serial.println();
}

void Control::RunHelpSpeed(void)
{
    Serial.println("speed [OPTIONS]");
    Serial.println("Change speed of hexapod movements.");
    Serial.println("-r");
    Serial.println("    rise speed");
    Serial.println("-d");
    Serial.println("    decrease speed");
    Serial.println("-c");
    Serial.println("    change step of rise/decrease (speed is [0,255] range)");
    Serial.println("Example of use: speed -c30; ");
    Serial.println();
}

void Control::RunHelpHeight(void)
{
    Serial.println("height [OPTIONS]");
    Serial.println("Change height of hexapod.");
    Serial.println("-r");
    Serial.println("    rise height");
    Serial.println("-d");
    Serial.println("    decrease height");
    Serial.println("-c");
    Serial.println("    change step of rise/decrease (height is in milimeters)");
    Serial.println("Example of use: height -c30; ");
    Serial.println();
}

void Control::RunHelpFootDistance(void)
{
    Serial.println("foot_distance [OPTIONS]");
    Serial.println("Change foot distance of hexapod legs.");
    Serial.println("-r");
    Serial.println("    rise foot distance");
    Serial.println("-d");
    Serial.println("    decrease foot distance");
    Serial.println("-c");
    Serial.println("    change step of rise/decrease (foot distance is in milimeters)");
    Serial.println("Example of use: foot distance -c30; ");
    Serial.println();
}

void Control::RunHelpLog(void)
{
    Serial.println("log [OPTIONS]");
    Serial.println("Change information given by the log.");
    Serial.println("-1 \t");
    Serial.println("    error messages");
    Serial.println("-2\t");
    Serial.println("    error and warn messages");
    Serial.println("-3\t");
    Serial.println("    error, warn and info messages");
    Serial.println("Example of use: log -2; ");
    Serial.println("-4\t");
    Serial.println("    error, warn, info and debug messages");
    Serial.println("Example of use: log -2; ");
    Serial.println();
}

void Control::RunHelpCalibration(void)
{
    Serial.println("calibrate(Leg,Member,Angle)");
    Serial.println("Calibration of the indicated leg.");
    Serial.println("Leg: number of the leg [0 - 5].");
    Serial.println("Member: Coxa = 0, Femur = 1, Tibia = 2 [0 - 2].");
    Serial.println("Angle: angle to default position [0 - 120].");
    Serial.println("NOTE: to complete calibration send angle 255.");
    Serial.println("Example of use: calibrate(3,0,90);");
    Serial.println();
}

void Control::RunDefault(void){
    Serial.print(_command);
    Serial.println(": not correct.Try help.");
}

#endif
