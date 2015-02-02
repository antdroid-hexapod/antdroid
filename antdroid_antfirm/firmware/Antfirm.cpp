/* Antfirm.cpp: main hexapod program.
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

#include "Antfirm.h"

byte level_log = 0;

#ifdef ControlRos
    ros::NodeHandle arduino;
#endif

Hexapod Antdroid;

Control DefaultControl(&Antdroid);

void setup() {

    DefaultControl.Start();
	Antdroid.Start();

}

void loop() {
	DefaultControl.ReadInput();
	
}
