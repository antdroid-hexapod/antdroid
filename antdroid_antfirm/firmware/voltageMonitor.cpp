/* voltageMonitor.cpp: core of voltage monitor.
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

#include "voltageMonitor.h"


uint8_t readVoltage(void)
{
    #ifdef VoltageInPin
    int voltage_divisor = 0;
    uint8_t real_voltage1d = 0;

    voltage_divisor = analogRead(VoltageInPin);

    real_voltage1d = (((long)voltage_divisor * (R1 + R2) / R2) * 5 * 10 )/ 1023
    
    log("Voltage:", real_voltage1d, Debug);

    return real_voltage1d;

    #else
    return 120;

    #endif
}