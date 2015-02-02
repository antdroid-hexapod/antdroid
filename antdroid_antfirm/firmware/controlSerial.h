/* controlSerial.h: serial control for hexapods
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

#include "hexapod.h"
#include "Arduino.h"
#include "log.h"

#define DefaultGaitMsg "Default gait: tripod."

extern byte level_log;

class Control
{
    public:
    Control(Hexapod* Antdroid);
    
    void Start(void);

    void ReadInput(void);


    private:

    char _current_char;
    String _command;
    Hexapod* _antdroid;

    void RunCommand(void);

    bool TryRunMove(byte endCommandName);
    bool TryRunTurn(byte endCommandName);
    bool TryRunGait(byte endCommandName);
    bool TryRunBalance(byte endCommandName);
    bool TryRunSpeed(byte endCommandName);
    bool TryRunHeight(byte endCommandName);
    bool TryRunFootDistance(byte endCommandName);
    bool TryChangeLog(byte endCommandName);
    bool TryChangeCalibration(byte endCommandName);

    void RunHelp(void);
    void RunHelpMove(void);
    void RunHelpTurn(void);
    void RunHelpGait(void);
    void RunHelpBalance(void);
    void RunHelpSpeed(void);
    void RunHelpHeight(void);
    void RunHelpFootDistance(void);
    void RunHelpLog(void);
    void RunHelpCalibration(void);

    void RunDefault(void);

};


#endif
