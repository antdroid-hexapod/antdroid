/* trig.h: Trigonometric functions optimized for Inverse Kinematic in hexapods
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

#ifndef trig_h
#define trig_h

#include <Arduino.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

/*
 * Default angle values:
 *    Degree    One decimal. Example: 1800 = 180.0 º
 *    Radians   Four decimal. Example: 31416 = 3,1416 rad
 */


#define	Shift1Decimal 10
#define	Shift2Decimal 100
#define	Shift3Decimal 1000
#define	Shift4Decimal 10000
#define	Shift6Decimal 1000000

#define Pi 31416

short GetCos(short AngleDeg1);

short GetSin(short AngleDeg1);

short GetArcCos(short cos4);

short GetArcTan (short AtanX, short AtanY, long XYhyp);

short Hypotenuse (short x, short y);

unsigned long Isqrt (unsigned long n) ;

short Rad2Deg(short AngleRad4);

#endif