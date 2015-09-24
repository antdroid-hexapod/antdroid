/* configuration.h: defined parameter of hexapod
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

// Axis system: X - front, Y - left, Z - top
// We assume that hexapod dimensions are simetrical in X and Y axis.
#ifndef configuration_h
#define configuration_h

// ****************************************************************************
// MOTHERBOARD
// select your board: BOARD_PROTOTIPE or BOARD_ANTDROID supported
#define MOTHERBOARD BOARD_PROTOTIPE

// ****************************************************************************
// BATTERY
// Cutoff voltage with 1 decimal: 100 = 10v 
#define BATTERY_CUTOFF_VOLTAGE 100

// ****************************************************************************
// MIN & MAX ANGLES
// Angle expresed with 1 decimal : 90.0 = 900
#define LeftFemurMin -900
#define LeftFemurMax 100
#define LeftTibiaMin -750
#define LeftTibiaMax 750

#define RightFemurMin -LeftFemurMax
#define RightFemurMax -LeftFemurMin
#define RightTibiaMin -LeftTibiaMax
#define RightibiaMax -LeftTibiaMin

#define LeftFrontCoxaMin RightFrontCoxaMin
#define LeftFrontCoxaMax RightFrontCoxaMax

#define RightFrontCoxaMin -300
#define RightFrontCoxaMax 300

#define LeftMiddleCoxaMin RightMiddleCoxaMin
#define LeftMiddleCoxaMax RightMiddleCoxaMax

#define RightMiddleCoxaMin -300
#define RightMiddleCoxaMax 300

#define LeftRearCoxaMin	RightRearCoxaMin
#define LeftRearCoxaMax	RightRearCoxaMax

#define RightRearCoxaMin -300
#define RightRearCoxaMax 300

// ****************************************************************************
// LEG DIMENSIONS in mm
#define CoxaLength 54
#define FemurLength 70
#define TibiaLength 155

// ****************************************************************************
// BODY DIMENSIONS in mm
#define CoxaOffsetX 78

#define CoxaOffsetY 55
#define MiddleCoxaOffsetY 78

// ****************************************************************************
// BODY DEFAULT ANGLE referred to axis X
// Angle expresed with 1 decimal : 90.0 = 900
#define LeftFrontCoxaDefaultAngle 450
#define RightFrontCoxaDefaultAngle -LeftFrontCoxaDefaultAngle
#define LeftMiddleCoxaDefaultAngle 900
#define RightMiddleCoxaDefaultAngle -900
#define LeftRearCoxaDefaultAngle 1350
#define RightRearCoxaDefaultAngle -LeftRearCoxaDefaultAngle




// ****************************************************************************
// LEG DEFFAULT POSSITION referenced leg-axis
#define FootDistance 120
#define	FootHeight -100
#define DefaultTime 2000
#define Gap 20

// ****************************************************************************
// SERVO SPEED degrees per second (º/s)
// MG996r datasheet:
// 	- 0.17 s/60º 4.8v without load = 353º/s
// 	- 0.13 s/60º 6.8v without load = 461º/s 
#define MaxRealSpeed 370

// ****************************************************************************
// BODY MOVEMENT SPEED
// value [0,255] -> 255 = MaxSpeed
#define DefaultSpeed 80

// ****************************************************************************
// BODY ROTATION SPEED 
// balanceSpeed = _speed * BalanceSpeedProportion
#define BalanceSpeedProportion 0.85

// ****************************************************************************
// CONTROL
// ControlSerial    control hexapod by serial port
// ControlRos       control hexapod like a ROS node
#define ControlRos

// ****************************************************************************
// DEFAULT CONTROL STEPS
#define FootDistanceStep 10
#define SpeedStep 10
#define FloorHeightStep 10

#endif