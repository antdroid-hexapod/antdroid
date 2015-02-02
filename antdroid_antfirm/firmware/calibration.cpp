/* calibration.cpp: generic calibration functions.
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
 * Allows us to have a default calibration before we start to run the
 * Antfirm code for the 18 servos que have attached to the Arduino
 * MEGA board. This calibration is saved in the internal EEPROM memory
 * of the board. For debug and testing proves, we have added a function
 * to know the actual values of the memory

 * The functions are:

 * currentCalibration(): send by serial port all the calibration angles.
 * writeCalibrationCompleted(): write byte in eeprom to define calibration
 * completed.
 * tryCalibrationCompleted(): return true if calibration completed
 */

#include "calibration.h"

#include <avr/eeprom.h>

void currentCalibration(void)
{
    log("Current calibration of the servos: ", Info);

    for(byte i=0; i<6; i++)
    {
        log("Leg:", i, Info);
        log("------------------------------------------------", Info);
        for(byte j=0; j<3; j++)
        {            
            if(j==0)
                log("Coxa:", Info);
            else if(j==1)
                log("Femur:", Info);
            else
                log("Tibia:", Info);

            log(" ", eeprom_read_byte((unsigned char *)(j+3*i)), Info);
            log("EEPROM dir:", j+3*i, Debug);
        }
    log(" ", Info);
    }
}

void writeCalibrationCompleted(void)
{
    eeprom_write_byte((unsigned char *) (CalibrationCompletedDir),
        CalibrationCompletedValue);
    log("Calibration completed", Info);
    log("You must restart your Arduino now", Info);
}

bool tryCalibrationCompleted(void)
{
    if(eeprom_read_byte((unsigned char *) (CalibrationCompletedDir)) ==
        CalibrationCompletedValue)
        return true;

    log("Calibration not completed yet",Error);
    log("To complete calibration send calibrate with angle = 255",Info);

    return false;
}